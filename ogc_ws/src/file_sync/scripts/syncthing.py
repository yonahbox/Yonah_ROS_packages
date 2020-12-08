#!/usr/bin/env python3

# Copyright (C) 2020 Rumesh Sudhaharan

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rospy
from std_msgs.msg import String, UInt8
from identifiers.srv import SetDetails, GetId, GetIdRequest

import requests as req
import xml.etree.ElementTree as xml
from pathlib import Path
import subprocess
import json

class Syncthing:
	def __init__(self):
		self.host = "http://localhost:8384"

		rospy.wait_for_service("identifiers/set/st_id")
		rospy.wait_for_service("identifiers/get/id")

		self.parse()

		self._error_pub = rospy.Publisher("ogc/to_despatcher/error", String, queue_size=5)
		self._connected_pub = rospy.Publisher("ogc/files/connected", UInt8, queue_size=5)
		self._disconnected_pub = rospy.Publisher("ogc/files/disconnected", UInt8, queue_size=5)
		self._connected_pub_str = rospy.Publisher("ogc/files/connected/string", String, queue_size=5)
		self._disconnected_pub_str = rospy.Publisher("ogc/files/disconnected/string", String, queue_size=5)

		self.set_device = rospy.ServiceProxy("identifiers/set/st_id", SetDetails)
		self.get_device = rospy.ServiceProxy("identifiers/get/id", GetId)

		self.set_id()

	def parse(self):
		home_dir = str(Path.home())
		config_path = home_dir + "/.config/syncthing/config.xml"
		if not Path(config_path).is_file():
			print("Error: file not found")
			exit()

		root = xml.parse(config_path)
		for elem in root.iter('apikey'):
			self.api_key = elem.text

	def set_id(self):
		try:
			sub_call = subprocess.run(["syncthing", "-device-id"], capture_output=True, text=True)
			self.device_id = sub_call.stdout.rstrip()
			self.set_device(self.device_id)
		except FileNotFoundError:
			rospy.logwarn("Unable to call syncthing")

	def _post(self, url):
		try:
			req.post(self.host + url, headers = {
				"X-API-Key": self.api_key
			})
		except req.exceptions.RequestException:
			self._error_pub.publish("syncthing not running")

	def _get(self, url):
		try:
			result = req.get(self.host + url, headers={
				"X-API-Key": self.api_key
			})
		except req.exceptions.RequestException:
			self._error_pub.publish("syncthing not running")
			return None

		return result.json()

	def pause(self):
		self._post("/rest/system/pause")

	def resume(self):
		self._post("/rest/system/resume")

	def event_subscribe(self):
		last_id = 0
		while True:
			rospy.loginfo("MAKING REQUEST")
			response = self._get("/rest/events?events=DeviceConnected,DeviceDisconnected&limit=1&since="+str(last_id))
			
			if response is None:
				return

			print(json.dumps(response, sort_keys=True, indent=4))
			if len(response) == 0:
				continue

			last_id = response[0]["id"]
			device_id = response[0]["data"]["id"]

			id_request = GetIdRequest()
			id_request.type = 5
			id_request.data = device_id

			system_id = self.get_device(id_request)

			if response[0]["type"] == "DeviceConnected":
				self._connected_pub_str.publish(device_id) if system_id.id == 0 else self._connected_pub.publish(system_id.id)
			elif response[0]["type"] == "DeviceDisconnected":
				self._disconnected_pub_str.publish(device_id) if system_id.id == 0 else self._disconnected_pub.publish(system_id.id)
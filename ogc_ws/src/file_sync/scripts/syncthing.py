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
from std_msgs.msg import String

import requests as req
import xml.etree.ElementTree as xml
from pathlib import Path
import subprocess
import json

class Syncthing:
	def __init__(self):
		self.host = "http://localhost:8384"
		self.parse()

		self._error_pub = rospy.Publisher("ogc/to_despatcher/error", String, queue_size=5)
		self._connected_pub = rospy.Publisher("ogc/files/connected", String, queue_size=5)
		self._disconnected_pub = rospy.Publisher("ogc/files/disconnected", String, queue_size=5)


	def parse(self):
		home_dir = str(Path.home())
		config_path = home_dir + "/.config/syncthing/config.xml"
		if not Path(config_path).is_file():
			print("Error: file not found")
			exit()

		root = xml.parse(config_path)
		for elem in root.iter('apikey'):
			self.api_key = elem.text

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

			device_id = response[0]["data"]["id"]

			if response[0]["type"] == "DeviceConnected":
				self._connected_pub.publish(device_id)
			elif response[0]["type"] == "DeviceDisconnected":
				self._disconnected_pub.publish(device_id)
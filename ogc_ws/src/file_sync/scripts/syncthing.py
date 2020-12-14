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
	def __init__(self, error_pub=None):
		self.host = "http://localhost:8384"
		self._error_pub = error_pub

		self.parse_config()

	# Parse the syncthing config file
	def parse_config(self):
		home_dir = str(Path.home())
		config_path = home_dir + "/.config/syncthing/config.xml"
		if not Path(config_path).is_file():
			print("Error: file not found")
			exit()

		root = xml.parse(config_path)
		for elem in root.iter('apikey'):
			self.api_key = elem.text

	# get syncthing id
	def get_id(self):
		try:
			sub_call = subprocess.run(['syncthing', '-device-id'], capture_output=True, text=True)
			return sub_call.stdout.rstrip()
		except FileNotFoundError:
			print("Unable to call syncthing, please check if it is installed")

	# Make a POST request to syncthings REST API
	def _post(self, url, data):
		try:
			req.post(self.host + url, headers = {
				"X-API-Key": self.api_key
			}, data=json.dumps(data))
		except req.exceptions.RequestException:
			print("POST ERROR")
			self._error_pub.publish("syncthing not running")

	# Make a GET request to syncthings REST API
	def _get(self, url):
		try:
			result = req.get(self.host + url, headers={
				"X-API-Key": self.api_key
			})
		except req.exceptions.RequestException:
			self._error_pub.publish("syncthing not running")
			return None

		return result.json()

	# pause syncthing
	def pause(self):
		self._post("/rest/system/pause", None)

	# resume syncthing
	def resume(self):
		self._post("/rest/system/resume", None)

	# Listen for events in syncthing
	def get_event(self, last_id):
		response = self._get("/rest/events?events=DeviceConnected,DeviceDisconnected&limit=1&since="+str(last_id))
		
		if not response or len(response) == 0:
			return None
		return response[0]

	# get list of connected devices in syncthing
	def get_devices(self):
		devices = self._get("/rest/config/devices")

		known_devices = []
		for device in devices:
			known_devices.append(device["deviceID"])

		return known_devices

	# add device to syncthing and add it to the default folder as well
	def add_device(self, st_id, label):
		self._post("/rest/config/devices", {
			"deviceID": st_id,
			"name": label
		})

		folder = self._get("/rest/config/folders/default")
		folder["devices"].append({
			"deviceID": st_id
		})

		self._post("/rest/config/folders", folder)

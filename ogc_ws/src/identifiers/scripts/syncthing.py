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
from identifiers.srv import SetDetails

import requests as req
import xml.etree.ElementTree as xml
from pathlib import Path
import subprocess

class Syncthing:
	def __init__(self):
		self.host = "http://localhost:8384"
		# rospy.wait_for_service("identifiers/get/st_id")
		rospy.wait_for_service("identifiers/set/st_id")

		self.parse()

		# self.get_device = rospy.ServiceProxy("identifiers/get/st_id")
		self.set_device = rospy.ServiceProxy("identifiers/set/st_id", SetDetails)

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
			rospy.logwarn("unable to call syncthing")

	def pause(self, id_n):
		# device = self.get_device(id_n)
		print("device="+id_n)
		result = req.post(self.host + "/rest/system/pause", data="device=" + id_n, headers={
			"X-API-Key": self.api_key
		})
		print(result)

	def resume(self, id_n):
		print("device="+id_n)
		print(self.api_key)
		result = req.post(self.host + "/rest/system/resume", data="device=" + id_n, headers={
			"X-API-Key": self.api_key
		})
		print(result)
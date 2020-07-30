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

import requests as req
import xml.etree.ElementTree as xml
from pathlib import Path
import subprocess

class Syncthing:
	def __init__(self):
		self.host = "http://localhost:8384"
		self.parse()
		self._error_pub = rospy.Publisher("ogc/to_despatcher/error", String)

	def parse(self):
		home_dir = str(Path.home())
		config_path = home_dir + "/.config/syncthing/config.xml"
		if not Path(config_path).is_file():
			print("Error: file not found")
			exit()

		root = xml.parse(config_path)
		for elem in root.iter('apikey'):
			self.api_key = elem.text

	def pause(self):
		result = req.post(self.host + "/rest/system/pause", headers={
			"X-API-Key": self.api_key
		})

	def resume(self):
		try:
			result = req.post(self.host + "/rest/system/resume", headers={
				"X-API-Key": self.api_key
			})
		except ConnectionRefusedError:
			self._error_pub.publish("syncthing not running")

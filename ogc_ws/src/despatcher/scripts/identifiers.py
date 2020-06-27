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

import json

# helper class to hold information about the devices specified in the identifiers file
class Device:
	def __init__(self, label, is_air, dev_id, number, imei, rb_serial):
		self.label = label
		self.is_air = is_air
		self.id = dev_id
		self.number = number
		self.imei = imei
		self.rb_serial = rb_serial

class Identifiers:
	def __init__(self, json_file, is_air, valid_ids):
		self.json_file = json_file	# Location of identifiers file
		self.is_air = is_air		# Boolean to know if it is running air side or ground side
		self.valid_ids = valid_ids	# List of whitelisted ids as defined the identifiers file

		self.whitelist = []			# List of whitelisted devices (contains instances of the Device class)
		self.whitelist_nums = []	# List of whitelisted numbers (contains phone number of the whitelisted devices)
		self.whitelist_imei = []	# List of whitelisted imei numbers (contains imei number for whitelisted rockblock modules) - will be changed to serial number
		
		self.rock7_un = ""			# username for rockblock
		self.rock7_pw = ""			# password for rockblock
		self.aws_url = ""			# url to AWS instance

		# parse the identifiers file
		self._parse_file()

	def _parse_file(self):
		with open(self.json_file) as file:
			try:
				# read the file as a json object
				self.json_obj = json.load(file)
			except JSONDecodeError:
				# file does not contain valid json
				print("invalid identifier file")
				exit()

		# For loop with ternary operator to decide which array to check from the json object
		# add all whitelisted devices into the whitelist
		for obj in (self.json_obj["ground"] if self.is_air else self.json_obj["air"]):
			if obj["id"] in self.valid_ids:
				self.whitelist.append(Device(obj["label"], self.is_air, obj["id"], obj["number"], obj["imei"], obj["rb_serial"]))
				self.whitelist_nums.append(obj["number"])
				self.whitelist_imei.append(obj["imei"])

		# for standalone numbers (not currently in use)
		for num in self.json_obj["standalone"]:
			self.whitelist_nums.append(num)

		self.rock7_un = self.json_obj["sbd_details"]["rock7_username"]
		self.rock7_pw = self.json_obj["sbd_details"]["rock7_password"]
		self.aws_url = self.json_obj["sbd_details"]["aws_url"]

	# refresh details from file, in case it has changed since the time the object was initialized
	def refresh_details(self):
		self._parse_file()

	# return the device associated with the id
	def get_device(self, id_n):
		for device in self.whitelist:
			if device.id == id_n:
				return device

	# return phone number associated with id if it is whitelisted
	def get_number(self, id_n):
		device = self.get_device(id_n)
		# returns the correct value only if the requested id was included in the initial whitelist
		return device.number if device else 0

	# returns a tuple containing sbd relevant information associated with id if it is whitelisted
	def get_sbd_info(self, id_n):
		device = self.get_device(id_n)
		# returns the correct value only if the requested id was included in the initial whitelist
		return (device.imei, device.rb_serial) if device else ()

	# return the whitelisted phone numbers
	def get_whitelist(self):
		return self.whitelist_nums

	# Does a lazy check to see if the received message is from a valid sender
	# Trusts that the sender of the message was correctly identified in the message headers
	# Can be fooled by imitating the message headers
	def is_valid_message_lazy(self, message):
		return int(message[2]) == (0 if self.is_air else 1) and int(message[4]) in self.valid_ids

	# link:
	#	0: telegram
	#	1: sms
	#	2: sbd
	def is_valid_message(self, link, type, details):
		if link == 0 or link == 1:
			return details in self.whitelist_nums
		elif link == 2:
			return details[0] in [x.imei for x in self.whitelist_devs] and details[1] in [y.rb_serial for y in self.whitelist_devs]
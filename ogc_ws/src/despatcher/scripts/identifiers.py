#!/usr/bin/env python3

import json

class Device:
	def __init__(self, label, is_air, dev_id, number, imei, rb_serial):
		self.label
		self.is_air = is_air
		self.id = dev_id
		self.number = number
		self.imei = imei
		self.rb_serial = rb_serial


class Devices:
	def __init__(self, json_file, valid_ids):
		self.json_file = json_file
		self.valid_ids = valid_ids
		self.whitelist_devs = []
		self.whitelist_nums = []
		self.rock7_un = ""
		self.rock7_pw = ""
		self.aws_url = ""

		_parse_file()

	def _parse_file(self):
		with open(self.json_file) as file:
			try:
				self.json_obj = json.load(file)
			except JSONDecodeError:
				print("invalid identifier file")
				exit()

		for obj in self.json_obj["identifiers"]:
			if obj["id"] in self.valid_ids:
				self.whitelist_devs.append(Device(obj["label"], obj["is_air"], obj["id"], obj["number"], obj["imei"], obj["rb_serial"]))
				self.whitelist_nums.append(obj["number"])

		for num in self.json_obj["standalone_numbers"]:
			self.whitelist_nums.append(num)

		self.rock7_un = self.json_obj["sbd_details"]["rock7_username"]
		self.rock7_pw = self.json_obj["sbd_details"]["rock7_password"]
		self.aws_url = self.json_obj["sbd_details"]["aws_url"]

	# link:
	#	0: telegram
	#	1: sms
	#	2: sbd
	def is_valid_message(self, link, details):
		if link == 0 or link == 1:
			if details in self.whitelist_nums:
				return True
			else:
				return False
		elif link == 2:
			if details[0] in [x.imei for x in self.whitelist_devs] and details[1] in [y.rb_serial for y in self.whitelist_devs]:
				return True
			else:
				return False

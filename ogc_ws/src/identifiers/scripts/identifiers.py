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
	def __init__(self, label, is_air, dev_id, number, imei, rb_serial, telegram_id, syncthing_id):
		self.label = label
		self.is_air = is_air
		self.id = dev_id
		self.number = number
		self.imei = imei
		self.rb_serial = rb_serial
		self.telegram_id = telegram_id
		self.syncthing_id = syncthing_id

	def to_json(self):
		return {
			"label": self.label,
			"is_air": self.is_air,
			"id": self.id,
			"number": self.number,
			"imei": self.imei,
			"rb_serial": self.rb_serial,
			"telegram_id": self.telegram_id,
			"syncthing_id": self.syncthing_id
		}

class Identifiers:
	def __init__(self, json_file, is_air, self_id_file, valid_ids_file):
		self.json_file = json_file			# Location of identifiers file
		self.is_air = is_air				# Boolean to know if it is running air side or ground side
		self.self_id = 0	 				# id number as defined in the identifiers file of the device this runs on
		self.self_device = None				# Information about the device this runs one
		self.valid_ids = []					# List of whitelisted ids as defined the identifiers file
		self.admin = False

		# read valid ids from the valid_ids file
		if valid_ids_file:
			try:
				with open(valid_ids_file, "r") as f:
					while valid_id := f.readline():
						self.valid_ids.append(int(valid_id))
			except FileNotFoundError:
				print("Valid ids file is not available")
				print("please check that the telegram_bone script works properly")
				exit()
		else:
			self.admin = True

		if not self.admin:
			try:
				with open(self_id_file, "r") as f:
					self.self_id = int(f.readline().rstrip())
			except FileNotFoundError:
				print("self_id file is not available")
				print("Please ensure the device was properly set up")
				exit()

		self.whitelist = []					# List of whitelisted devices (contains instances of the Device class)
		self.whitelist_nums = []			# List of whitelisted numbers (contains phone number of the whitelisted devices)
		self.whitelist_rb_serial=[]			# List of whitelisted rb serial numbers (contains serial number for whitelisted rockblock modules)
		self.whitelist_telegram_ids = []	# List of whitelisted telegram user ids
		self.syncthing_ids = []				# List of syncthing ids

		self.rock7_un = ""					# username for rockblock
		self.rock7_pw = ""					# password for rockblock
		self.svr_hostname = ""				# hostname of hosted web server
		self.svr_ip = ""					# ip address of hosted web server
		self.admin_id = 0

		self.air_devices = []
		self.ground_devices = []

		# used to prevent two functions writing at the same time
		# basically a mutex lock
		self.file_busy = False

		# parse the identifiers file
		self.parse_file()


	def parse_file(self):
		self.file_busy = True

		with open(self.json_file) as file:
			try:
				# read the file as a json object
				self.json_obj = json.load(file)
			except json.JSONDecodeError:
				# file does not contain valid json
				print("invalid identifier file")
				exit()

		# clear the whitelists to prevent duplicates
		self.whitelist.clear()
		self.whitelist_nums.clear()
		self.whitelist_rb_serial.clear()
		self.whitelist_telegram_ids.clear()
		self.syncthing_ids.clear()

		for obj in self.json_obj['ground']:
			self.ground_devices.append(Device(obj['label'], False, obj['id'], obj['number'], obj['imei'], obj['rb_serial'], obj['telegram_id'], obj.get("syncthing_id", None)))
			
			# add all available syncthing ids
			if obj.get('syncthing_id', None):
				self.syncthing_ids.append(obj['syncthing_id'])
				
			# whitelist all telegram ids if admin
			if self.admin:
				self.whitelist_telegram_ids.append(str(obj['telegram_id']))

		for obj in self.json_obj["air"]:
			self.air_devices.append(Device(obj['label'], True, obj['id'], obj['number'], obj['imei'], obj['rb_serial'], obj['telegram_id'], obj.get("syncthing_id", None)))
			# add all available syncthing ids
			if obj.get('syncthing_id', None):
				self.syncthing_ids.append(obj['syncthing_id'])

			# whitelist all telegram ids if admin
			if self.admin:
				self.whitelist_telegram_ids.append(str(obj['telegram_id']))

		# For loop with ternary operator to decide which array to check from the json object
		# add all whitelisted devices into the whitelist and required information to their respective whitelists
		for obj in self.ground_devices if self.is_air else self.air_devices:
			if obj.id in self.valid_ids:
				self.whitelist.append(obj)
				self.whitelist_nums.append(obj.number)
				self.whitelist_rb_serial.append(obj.rb_serial)
				self.whitelist_telegram_ids.append(str(obj.telegram_id))

		# for obj in (self.json_obj["ground"] if self.is_air else self.json_obj["air"]):
		# 	if obj["id"] in self.valid_ids:
		# 		self.whitelist.append(Device(obj["label"], self.is_air, obj["id"], obj["number"], obj["imei"], obj["rb_serial"], obj["telegram_id"], obj.get("syncthing_id", None)))
		# 		self.whitelist_nums.append(obj["number"])
		# 		self.whitelist_rb_serial.append(obj["rb_serial"])
		# 		if "telegram_id" in obj.keys():
		# 			self.whitelist_telegram_ids.append(str(obj["telegram_id"]))

		# Get details about the device this is running on
		for obj in (self.json_obj["air"] if self.is_air else self.json_obj["ground"]):
			if obj["id"] == self.self_id:
				self.self_device = Device(obj["label"], self.is_air, obj["id"], obj["number"], obj["imei"], obj["rb_serial"], obj["telegram_id"], obj.get("syncthing_id", None))
				break

		# for standalone numbers (not currently in use)
		# for num in self.json_obj["standalone"]:
		# 	self.whitelist_nums.append(num)

		# Get remaining information for SBD link
		self.rock7_un = self.json_obj["sbd_details"]["rock7_username"]
		self.rock7_pw = self.json_obj["sbd_details"]["rock7_password"]
		self.svr_hostname = self.json_obj["sbd_details"]["svr_hostname"]
		self.svr_ip = self.json_obj["sbd_details"]["svr_ip"]

		self.admin_id = self.json_obj["admin"]["telegram_id"]
		self.file_busy = False

	# return the device associated with the id from whitelisted devices
	def get_device(self, id_n):
		for device in self.whitelist:
			if device.id == id_n:
				return device

	# return the device associated with the id from any device
	def get_device_details(self, id_n, is_air):
		device_list = self.air_devices if is_air else self.ground_devices
		for dev in device_list:
			if dev.id == id_n:
				return dev

		return None

	# return phone number associated with id if it is whitelisted
	def get_number(self, id_n):
		device = self.get_device(id_n)
		# returns the correct value only if the requested id was included in the initial whitelist
		return device.number if device else None

	# return the telegram user id associated with an id if it is whitelisted
	def get_telegram_id(self, id_n):
		# check if id is valid
		device = self.get_device(id_n)
		if device is None:
			return None
		
		telegram_id = device.telegram_id
		return telegram_id

	# return the telegram_id of the admin account
	def get_admin_id(self):
		return self.admin_id

	# return the serial number associated with an id if it is whitelisted
	def get_sbd_serial(self, id_n):
		device = self.get_device(id_n)
		# returns the correct value only if the requested id was included in the initial whitelist
		return device.rb_serial if device else None
	
	# return the imei associated with an id if it is whitelisted
	def get_sbd_imei(self, id_n):
		device = self.get_device(id_n)
		# returns the correct value only if the requested id was included in the initial whitelist
		return device.imei if device else None

	# return credentials needed for sbd link
	def get_sbd_credentials(self):
		return self.rock7_un, self.rock7_pw, self.svr_hostname, self.svr_ip
	
	# return the whitelisted phone numbers
	def get_whitelist(self):
		return self.whitelist_nums

	# return the id of the device this run on
	def get_self_id(self):
		return self.self_id

	# return serial number for the device this runs on
	def get_self_serial(self):
		return self.self_device.rb_serial

	# return imei for the device this runs on
	def get_self_imei(self):
		return self.self_device.imei

	# return phone numebr of the device this runs on
	def get_self_number(self):
		return self.self_device.number

	# return the telegram_id associated with the specified id
	def get_air_telegram_id(self, id_n):
		for device in self.air_devices:
			if device.id == id_n:
				return device.telegram_id

	# return all syncthing ids available in identifiers
	def get_syncthing_ids(self):
		return self.syncthing_ids

	# return all the id numbers currently in use (device exists in identifiers file)
	def get_active_ids(self):
		air_ids = [dev["id"] for dev in self.json_obj["air"]]
		ground_ids = [dev["id"] for dev in self.json_obj["ground"]]
		return (air_ids, ground_ids)

	# return all valid ids
	def get_valid_ids(self):
		return self.valid_ids

	# def get_device(self, is_air, id_n):
		# for obj in self.json_obj["air"] if is_air

	# request adding a new device to admin
	def new_device_request(self, is_air, label, number, imei, rb_serial):
		device = {
			"request": "add",
			"label": label,
			"is_air": is_air,
			"number": number,
			"imei": imei,
			"rb_serial": rb_serial
		}
		return json.dumps(device)

	# request edit for a device to admin
	def edit_device_request(self, id_n, is_air, label="", number="", imei="", rb_serial=""):
		device = {
			"request": "edit",
			"label": label,
			"id": id_n,
			"is_air": is_air,
			"number": number,
			"imei": imei,
			"rb_serial": rb_serial
		}
		return json.dumps(device)

  # returns the system id for a certain device based on information type
	# data_type values:
	#	0: label
	#	1: number
	#	2: imei
	#	3: rb_serial
	#	4: telegram_id
	#	5: syncthing_id
	def get_system_id(self, data_type, data):
		for obj in self.air_devices + self.ground_devices:
			if data_type == 0 and data == obj.label:
				return obj.is_air, obj.id
			elif data_type == 1 and data == obj.number:
				return obj.is_air, obj.id
			elif data_type == 2 and data == obj.imei:
				return obj.is_air, obj.id
			elif data_type == 3 and data == obj.rb_serial:
				return obj.is_air, obj.id
			elif data_type == 4 and data == obj.telegram_id:
				return obj.is_air, obj.id
			elif data_type == 5 and data == obj.syncthing_id:
				return obj.is_air, obj.id

		print(f"Error {data_type} {data}")
		return 0, 0

  # add new device to the identifiers file
	def add_new_device(self, device):
		# get the correct array in the json object
		edit_list = self.json_obj["air"] if device["is_air"] else self.json_obj["ground"]

		# find an available id number
		selected_id = -1
		for i in range(1,256):
			if i not in [obj["id"] for obj in edit_list]:
				selected_id = i
				break

		# if no number is available (implies the maximum number of devices has been reached)
		if selected_id == -1:
			self.file_busy = False
			return False

		edit_list.append({
			"label": device["label"],
			"id": selected_id,
			"number": device["number"],
			"imei": device["imei"],
			"rb_serial": device["rb_serial"],
			"telegram_id": device["telegram_id"]
		})

		# write details to file
		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)
    
		self.parse_file()

		return selected_id


	# edit an existing device in the identifiers file
	def edit_device(self, device):	
		# get the correct array in the json object
		edit_list = self.json_obj["air"] if device["is_air"] else self.json_obj["ground"]

		# find the correct object for the id
		selected_device = None
		for dev in edit_list:
			if dev["id"] == device['id']:
				selected_device = dev
				break

		# id does not exist
		if selected_device is None:
			self.file_busy = False
			return False

		# edit the fields specified, keep old info if no new information provided
		selected_device["label"] = device["label"] if device["label"] != "" else selected_device["label"]
		selected_device["number"] = device["number"] if device["number"] != "" else selected_device["number"]
		selected_device["imei"] = device["imei"] if device["imei"] != ""else selected_device["imei"]
		selected_device["rb_serial"] = device["rb_serial"] if device["rb_serial"] != "" else selected_device["rb_serial"]
		selected_device["telegram_id"] = device["telegram_id"] if device["telegram_id"] != "" else selected_device["telegram_id"]
		selected_device["syncthing_id"] = device["syncthing_id"] if device["syncthing_id"] != "" else selected_device["syncthing_id"]

		#write details to file
		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)

		self.parse_file()

		return True

	def set_syncthing_id(self, device_id):
		if self.self_device.syncthing_id is not None:
			return False

		edit_list = self.json_obj["air"] if self.is_air else self.json_obj["ground"]

		selected_device = None

		for device in edit_list:
			if device["id"]  == self.self_id:
				selected_device = device
				break

		selected_device["syncthing_id"] = device_id

		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)

		self.parse_file()

		return True

	# Does a lazy check to see if the received message is from a valid sender
	# Trusts that the sender of the message was correctly identified in the message headers
	# Can be fooled by imitating the message headers
	def is_valid_message_lazy(self, message):
		return int(message[2]) == (0 if self.is_air else 1) and int(message[4]) in self.valid_ids

	# Does a proper check to see if the received message is from a valid sender
	# checks actual details from source (phone number, telegram user id and serial number depending on the link)
	def is_valid_sender(self, link, details):
		if link == 0:
			return details in self.whitelist_telegram_ids
		elif link == 1:
			return details[1:] in self.whitelist_nums
		elif link == 2:
			return details in self.whitelist_rb_serial
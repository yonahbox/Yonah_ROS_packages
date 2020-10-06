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
import rospy

from telegram.msg import ContactInfo

# helper class to hold information about the devices specified in the identifiers file
class Device:
	def __init__(self, label, is_air, dev_id, number, imei, rb_serial, telegram_id):
		self.label = label
		self.is_air = is_air
		self.id = dev_id
		self.number = number
		self.imei = imei
		self.rb_serial = rb_serial
		self.telegram_id = telegram_id

# Class to keep track of subscriptions to any topic when publishing
class TopicSubscriberNotification:
	def __init__(self, callback):
		self.callback = callback	# the callback function that will be called when the subscriber count is 1
		self.subscriber_count = 0

	def peer_subscribe(self, topic_name, topic_publish, peer_publish):
		rospy.loginfo("SUBSCRIBED to %s", topic_name)
		self.subscriber_count += 1
		if self.subscriber_count == 1:
			self.callback()

	def peer_unsubscribe(self, topic_name, num_peers):
		rospy.loginfo("UNSUBSCRIBED from %s", topic_name)
		self.subscriber_count -= 1


class Identifiers:
	def __init__(self, json_file, is_air, self_id, valid_ids):
		self.json_file = json_file			# Location of identifiers file
		self.is_air = is_air				# Boolean to know if it is running air side or ground side
		self.self_id = self_id 				# id number as defined in the identifiers file of the device this runs on
		self.valid_ids = valid_ids			# List of whitelisted ids as defined the identifiers file
		self.self_device = None				# Information about the device this runs one

		self.whitelist = []					# List of whitelisted devices (contains instances of the Device class)
		self.whitelist_nums = []			# List of whitelisted numbers (contains phone number of the whitelisted devices)
		self.whitelist_rb_serial=[]			# List of whitelisted rb serial numbers (contains serial number for whitelisted rockblock modules)
		self.whitelist_telegram_ids = []	# List of whitelisted telegram user ids

		self.rock7_un = ""					# username for rockblock
		self.rock7_pw = ""					# password for rockblock
		self.svr_hostname = ""				# hostname of hosted web server
		self.svr_ip = ""					# ip address of hosted web server

		# instance of the class to keep track of subscribers to a topic
		topic_cb = TopicSubscriberNotification(self.request_telegram_id)

		# topic publisher to the telegram node to add contacts
		self.telegram_add_contact = rospy.Publisher('ogc/to_telegram/contact', ContactInfo, queue_size=10, subscriber_listener=topic_cb)
		self.telegram_unknown_ids = []		# List of unknown telegram users (contains tuple of the form (label, numer))

		# parse the identifiers file
		self._parse_file()
		self.file_busy = False

	def _parse_file(self):
		with open(self.json_file) as file:
			try:
				# read the file as a json object
				self.json_obj = json.load(file)
			except JSONDecodeError:
				# file does not contain valid json
				print("invalid identifier file")
				exit()

		# clear the whitelists to prevent duplicates
		self.whitelist.clear()
		self.whitelist_nums.clear()
		self.whitelist_rb_serial.clear()
		self.whitelist_telegram_ids.clear()

		# Search file for all devices without a telegram user id
		for obj in self.json_obj["ground"] + self.json_obj["air"]:
			if "telegram_id" not in obj.keys():
					self.telegram_unknown_ids.append((obj["label"], obj["number"]))

		# For loop with ternary operator to decide which array to check from the json object
		# add all whitelisted devices into the whitelist and required information to their respective whitelists
		for obj in (self.json_obj["ground"] if self.is_air else self.json_obj["air"]):
			if obj["id"] in self.valid_ids:
				self.whitelist.append(Device(obj["label"], self.is_air, obj["id"], obj["number"], obj["imei"], obj["rb_serial"], obj.get("telegram_id", None)))
				self.whitelist_nums.append(obj["number"])
				self.whitelist_rb_serial.append(obj["rb_serial"])
				if "telegram_id" in obj.keys():
					self.whitelist_telegram_ids.append(str(obj["telegram_id"]))

		# Get details about the device this is running on
		for obj in (self.json_obj["air"] if self.is_air else self.json_obj["ground"]):
			if obj["id"] == self.self_id:
				self.self_device = Device(obj["label"], self.is_air, obj["id"], obj["number"], obj["imei"], obj["rb_serial"], obj.get("telegram_id", None))
				break

		# for standalone numbers (not currently in use)
		for num in self.json_obj["standalone"]:
			self.whitelist_nums.append(num)

		# Get remaining information for SBD link
		self.rock7_un = self.json_obj["sbd_details"]["rock7_username"]
		self.rock7_pw = self.json_obj["sbd_details"]["rock7_password"]
		self.svr_hostname = self.json_obj["sbd_details"]["svr_hostname"]
		self.svr_ip = self.json_obj["sbd_details"]["svr_ip"]

	# callback function for the topic subscriber class
	# used to add contacts to telegram after the telegram node has subscribed to the topic
	def request_telegram_id(self):
		rospy.loginfo("Telegram link has subscribed to contact topic")
		# loop through list of unknown devices
		while self.telegram_unknown_ids:
			contact = self.telegram_unknown_ids.pop(0)		# removed item from list as it wont be needed anymore
			self.update_telegram_id(contact[0], contact[1])	# request telegram to add the information to a new contact

	# return the device associated with the id from whitelisted devices
	def get_device(self, id_n):
		for device in self.whitelist:
			if device.id == id_n:
				return device

	# return the device associated with the id from any device
	def get_device_details(self, id_n, is_air):
		device_list = self.json_obj["air"] if is_air else self.json_obj["ground"]
		for dev in device_list:
			if dev["id"] == id_n:
				return Device(dev["label"], is_air, id_n, dev["number"], dev["imei"], dev["rb_serial"], dev.get("telegram_id", None))

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
		
		# check if device has the telegram id available, if not available, try to add it
		telegram_id = device.telegram_id
		if telegram_id is None:
			self.update_telegram_id(device.label, device.number)
			return None

		return telegram_id

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

	# return serial number for the device this runs on
	def get_self_serial(self):
		return self.self_device.rb_serial

	# return imei for the device this runs on
	def get_self_imei(self):
		return self.self_device.imei

	def get_self_number(self):
		return self.device.number

	# return all the id numbers currently in use (device exists in identifiers file)
	def get_active_ids(self):
		air_ids = [dev["id"] for dev in self.json_obj["air"]]
		ground_ids = [dev["id"] for dev in self.json_obj["ground"]]
		return (air_ids, ground_ids)

	# add new device to the identifiers file
	def add_new_device(self, is_air, label, number, imei, rb_serial):
		# get the correct array in the json object
		edit_list = self.json_obj["air"] if is_air else self.json_obj["ground"]

		# Check if there is space for more devices
		if len(edit_list) > 9:
			return False

		# find an available id number
		selected_id = -1
		for i in range(1,10):
			if i not in [obj["id"] for obj in edit_list]:
				selected_id = i
				break

		# if no number is available (implies the maximum number of devices has been reached)
		if selected_id == -1:
			return False

		edit_list.append({
			"label": label,
			"id": selected_id,
			"number": number,
			"imei": imei,
			"rb_serial": rb_serial
		})

		# write details to file
		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)

		self._parse_file()

		# get the telegram user id
		self.update_telegram_id(label, number)
		return True

	# edit an existing device in the identifiers file
	def edit_device(self, id_n, is_air, label="", number="", imei="", rb_serial=""):
		# get the correct array in the json object
		edit_list = self.json_obj["air"] if is_air else self.json_obj["ground"]

		# find the correct object for the id
		selected_device = None
		for device in edit_list:
			if device["id"] == id_n:
				selected_device = device
				break

		# id does not exist
		if selected_device is None:
			return False

		# edit the fields specified, keep old info if no new information provided
		selected_device["label"] = label if label != "" else selected_device["label"]
		selected_device["number"] = number if number != "" else selected_device["number"]
		selected_device["imei"] = imei if imei != ""else selected_device["imei"]
		selected_device["rb_serial"] = rb_serial if rb_serial != "" else selected_device["rb_serial"]

		#write details to file
		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)

		self._parse_file()

		# add the new number to telegram if number changed
		if number != "":
			self.update_telegram_id(selected_device["label"], selected_device["number"])
		return True

	# write the telegram user id for a device into the identifiers file
	def add_telegram_id(self, number, telegram_id):
		# edit objects in both air and ground if it exists in both (mainly needed in testing, unlikely to be needed in ops)
		obj_edited = False
		for device in self.json_obj["air"]:
			if device["number"] == number:
				if device.get("telegram_id", 0) != telegram_id:
					device["telegram_id"] = telegram_id
					obj_edited = True
					break

		for device in self.json_obj["ground"]:
			if device["number"] == number:
				if device.get("telegram_id", 0) != telegram_id:
					device["telegram_id"] = telegram_id
					obj_edited = True
					break

		if not obj_edited:
			return False

		rospy.loginfo("Adding telegram id %s to number %s", telegram_id, number)

		with open(self.json_file, "w") as f:
			json.dump(self.json_obj, f)

		self._parse_file()
		return True

	# request telegram to add contact and get the user id
	def update_telegram_id(self, label, number):
		rospy.loginfo("requesting telegram_id for %s (%s)", label, number)
		contact = ContactInfo()
		contact.label = label
		contact.number = number
		
		# check if the number is for the telegram account itself
		# it is not possible to add a contact of yourself in telegram
		if number == self.self_device.number:
			contact.me = True
		else:
			contact.me = False

		self.telegram_add_contact.publish(contact)


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
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

from ctypes import *
import json

from identifiers import Identifiers

class Td():
	def __init__(self, tdlib_dir, identifiers_loc, is_air, whitelist_devs):
		# Creates interface for libtdjson. This is a C library so it requires some initial setup
		tdjson_path = '/usr/local/lib/libtdjson.so.1.6.0'
		tdjson = CDLL(tdjson_path)

		client_create = tdjson.td_json_client_create
		client_create.restype = c_void_p
		client_create.argtypes = []
		self.client = client_create()

		self._client_receive = tdjson.td_json_client_receive
		self._client_receive.restype = c_char_p
		self._client_receive.argtypes = [c_void_p, c_double]

		self._client_send = tdjson.td_json_client_send
		self._client_send.restype = None
		self._client_send.argtypes = [c_void_p, c_char_p]
		
		self._client_execute = tdjson.td_json_client_execute
		self._client_execute.restype = c_char_p
		self._client_execute.argtypes = [c_void_p, c_char_p]
		
		self._client_destroy = tdjson.td_json_client_destroy
		self._client_destroy.restype = None
		self._client_destroy.argtypes = [c_void_p]

		fatal_error_callback_type = CFUNCTYPE(None, c_char_p)

		set_log_fatal_error_callback = tdjson.td_set_log_fatal_error_callback
		set_log_fatal_error_callback.restype = None
		set_log_fatal_error_callback.argtypes = [fatal_error_callback_type]

		fatal_error_cb = fatal_error_callback_type(self._fatal_error_cb)
		set_log_fatal_error_callback(fatal_error_cb)

		# Setup the Td class itself
		self.tdlib_dir = tdlib_dir				# Location fo the telegram client created by the tele_auth script
		self.is_air = is_air					# Boolean to know if it is running in the air side or ground side
		self.whitelist_devs = whitelist_devs	# whitelisted ids as defined in the identifiers file
		self.chat_list = []						# List of whitelisted chats
		self.command_user_ids = []				# List of user ids to easily check if sender is whitelisted
		
		# Initialize identifiers class to handle whitelisting
		self._ids = Identifiers(identifiers_loc, self.is_air, self.whitelist_devs)

		# Create a new isntance of the Chat class for each whitelisted device
		for num in self.whitelist_devs:
			self.chat_list.append(Chat(num, self._ids.get_number(num)))

		# Search the recently contacted chats for whitelisted numbers
		self.get_chats()

		# Reduce verbosity to reduce cluttering the log
		self._execute({
			'@type': 'setLogVerbosityLevel',
			'new_verbosity_level': 1
		})

	# Called when tdlib faces a fatal error
	def _fatal_error_cb(self, error):
		print('TDLiB fatal error: ', error)

	# Called for interacting with the telegram account
	def _execute(self, query):
		query = json.dumps(query).encode('utf-8')
		result = self._client_execute(None, query)
		if result:
			return json.loads(result.decode('utf-8'))
	
	# Get chat id associated with a specific id number as defined in the identifiers file
	def _get_chat_id(self, id_n):
		for chat in self.chat_list:
			if chat.whitelist_id == id_n:
				return chat.chat_id
		return False

	# Send request to telegram
	def send(self, query):
		query = json.dumps(query).encode('utf-8')
		self._client_send(self.client, query)

	# Wrapper to send text message to a specific id
	def send_message(self, id_n, msg):
		chat_id = self._get_chat_id(id_n)
		if not chat_id:
			print("invalid id specified")
			return False

		self.send({
			'@type': 'sendMessage',
			'chat_id': chat_id,
			'input_message_content': {
				'@type': 'inputMessageText',
				'text': {
					'@type': 'formattedText',
					'text': msg,
				}
			},
			'@extra': 'sent from Td.py'
		})

	# Wrapper to send text messages to multiple numbers
	def send_message_multi(self, ids, msg):
		for id_n in ids:
			self.send_message(id_n, msg)

	# Wrapper to send an image to a specific number
	def send_image(self, id_n, path):
		if not self.setup_complete():
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self._get_chat_id(chat_id),
				'input_message_content': {
					'@type': 'inputMessagePhoto',
					'photo': {
						'@type': 'inputFileLocal',
						'path': path
					}
				}
			})

	# Wrapper to send a video to a specific number
	def send_video(self, id_n, path):
		if not self.setup_complete():
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self._get_chat_id(chat_id),
				'input_message_content': {
					'@type': 'inputMessageVideo',
					'video': {
						'@type': 'inputFileLocal',
						'path': path
					}
				}
			})

	# Wrapper to send a gps location to a specific number
	# 	coordinates is expected to be a tuple of (latitude, longitude)
	def send_location(self, id_n, title, coordinates):
		if not self.setup_complete():
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self.sel_get_chat_id(chat_id),
				'input_message_content': {
					'@type': 'inputMessageVenue',
					'venue': {
						'@type': 'venue',
						'title': title,
						'location': {
							'@type': 'location',
							'latitude': coordinates[0],
							'longitude': coordinates[1]
						}
					}
				}
			})

	# Get the 10 most recent chats	
	def get_chats(self):
		self.send({
			'@type': 'getChats',
			'limit': 10
		})

	# Get specific information about a chat using its chat id
	def _get_chat_info(self, chat_id):
		self.send({
			'@type': 'getChat',
			'chat_id': chat_id
		})

	# receive information from telegram
	def receive(self):
		result_orig = self._client_receive(self.client, 1.0)
		if result_orig:
			# convert received string to a json dictionary
			result = json.loads(result_orig.decode('utf-8'))
			recv_type = result['@type']

			# handles authorization with the telegram account
			if recv_type == 'updateAuthorizationState':
				auth_state = result['authorization_state']['@type']
				if auth_state == 'authorizationStateWaitTdlibParameters':
					self.send({
						'@type': 'setTdlibParameters',
						'parameters': {
							'database_directory': self.tdlib_dir,
							'use_message_database': True,
							'use_secret_chats': True,
							'api_id': 1111226,
							'api_hash': '9996b83ac27902d79ce9486e6f740a08',
							'system_language_code': 'en',
							'device_model': 'Desktop',
							'system_version': 'Linux',
							'application_version': '1.0',
							'enable_storage_optimizer': True
						}
					})
				elif auth_state == 'authorizationStateWaitEncryptionKey':
					self.send({
						'@type': 'checkDatabaseEncryptionKey',
						'key': 'my_key' #need to change this
					})

			# this gives a list of chats
			elif recv_type == 'updateChatOrder':
				# checks if the chat has already been handled
				if result['chat_id'] not in [chat.chat_id for chat in self.chat_list]:
					self._get_chat_info(result['chat_id'])

			# This gives specific information about a chat
			elif recv_type == 'chat':
				# If it is a private chat, get information about the user
				if result['type']['@type'] == 'chatTypePrivate':
					self.send({
						'@type': 'getUser',
						'user_id': result['type']['user_id']
					})

			# This gives specific information about a user
			elif recv_type == 'user':
				for chat in self.chat_list:
					# Checks if the user is a whitelisted number
					if chat.phone_number == result["phone_number"]:
						# get relevant information about user
						chat.set_chat_id(result["id"])
						self.command_user_ids.append(result["id"])

			return result

	# destroy instance of libtdjson
	def destroy(self):
		self._client_destroy(self.client)
	
	# Check if the object has managed to locate all chats with the whitelisted numbers
	# returns Boolean
	def setup_complete(self):
		return all([chat.basic_complete for chat in self.chat_list])

# Helper class to store details about the chats in telegram
class Chat():
	def __init__(self, whitelist_id, phone_number):
		self.chat_id = 0
		self.whitelist_id = whitelist_id	# as defined in the identifiers file
		self.title = ""
		self.phone_number = phone_number

		# type of chat:
		#	0: unknown
		#	1: Private chat (PM)
		#	2: Group
		self.chat_type = 0

		# Only applicable for basic groups (chat type 2 above)
		self.basic_group_id = 0
		self.group_members = []
		self.basic_complete = False
	
	def set_chat_id(self, chat_id):
		self.chat_id = chat_id
		self.basic_complete = True

	def set_chat_type(self, chat_type):
		if chat_type == 'chatTypePrivate':
			self.chat_type = 1
		elif chat_type == 'chatTypeBasicGroup':
			self.chat_type = 2
			
	def set_title(self, title):
		self.title = title

	def add_member(self, member):
		self.group_members.append(member)

	def get_messages(self, n):
		pass
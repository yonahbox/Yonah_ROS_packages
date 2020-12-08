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

class Td():
	def __init__(self, tdlib_dir):
		# Creates interface for libtdjson. This is a C library so it requires some initial setup
		tdjson_path = '/usr/local/lib/libtdjson.so'
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

		# Reduce verbosity to reduce cluttering the log
		self._execute({
			'@type': 'setLogVerbosityLevel',
			'new_verbosity_level': 1
		})

		self.send({
			"@type": "setOption",
			"name": "ignore_inline_thumbnails",
			"value": {
				"@type": "optionValueBoolean",
				"value": True
			}
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
	
	# Get chats of user, latest 100 chats
	def get_chats(self):
		self.send({
			"@type": "getChats",
			"limit": 100
		})

	# Get the contacts of this telegram account
	def get_contacts(self):
		self.send({
			"@type": "getContacts",
		})

	# get information about the user
	def get_me(self):
		self.send({
			"@type": "getMe"
		})

	# Add single contact to telegram account
	# number should be a string of the form 6512345678
	def add_contact(self, number, label):
		self.send({
			"@type": "importContacts",
			"contacts": [{
				"@type": "contact",
				"first_name": label,
				"phone_number": "00" + number 	# telegram wants a 00 in front for some reason
			}]
		})

	# Send request to telegram
	def send(self, query):
		query = json.dumps(query).encode('utf-8')
		self._client_send(self.client, query)

	# Wrapper to send text message to a specific id
	def send_message(self, telegram_id, msg):
		self.send({
			'@type': 'sendMessage',
			'chat_id': telegram_id,
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

	def send_file(self, telegram_id, path, destination):
		self.send({
			'@type': 'sendMessage',
			'chat_id': telegram_id,
			'input_message_content': {
				'@type': 'inputMessageDocument',
				'document': {
					'@type': 'inputFileLocal',
					'path': path
				},
				'caption': {
					'@type': 'formattedText',
					'text': destination
				}
			}
		})

	# Wrapper to send an image to a specific number
	def send_image(self, telegram_id, path):
		self.send({
			'@type': 'sendMessage',
			'chat_id': telegram_id,
			'input_message_content': {
				'@type': 'inputMessagePhoto',
				'photo': {
					'@type': 'inputFileLocal',
					'path': path
				}
			}
		})

	# Wrapper to send a video to a specific number
	def send_video(self, telegram_id, path):
		self.send({
			'@type': 'sendMessage',
			'chat_id': telegram_id,
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
	def send_location(self, telegram_id, title, coordinates):
		self.send({
			'@type': 'sendMessage',
			'chat_id': telegram_id,
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

	def download_file(self, remote_id):
		self.send({
			'@type': 'downloadFile',
			'file_id': remote_id,
			'priority': 1,
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

			# This gives a list of user ids that are saved as contacts (in our usage)
			elif recv_type == "users":
				for user_id in result["user_ids"]:
					self.send({
						"@type": "getUser",
						"user_id": user_id
					})

			return result

	# Mark a message as read (double tick in app)
	def set_read(self, chat_id, message_id):
		# print('marking as read')
		self.send({
			'@type': 'viewMessages',
			'chat_id': chat_id,
			'message_ids': [message_id],
			'force_read': True	# This is currently needed to work
		})

	# destroy instance of libtdjson
	def destroy(self):
		self._client_destroy(self.client)
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
	def __init__(self, tdlib_dir, identifiers_loc, whitelist_devs):
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

		self.tdlib_dir = tdlib_dir
		self.whitelist_devs = whitelist_devs
		
		self._ids = Identifiers(identifiers_loc, self.whitelist_devs, [1])

		self.chat_list = []

		for num in self.whitelist_devs:
			self.chat_list.append(Chat(num, self._ids.get_number(True, num)))

		self.selected_chat = None

		self.get_chats()

		self.command_users = []
		self.command_user_ids = []
		

		self._execute({
			'@type': 'setLogVerbosityLevel',
			'new_verbosity_level': 1
		})

	def _fatal_error_cb(self, error):
		print('TDLiB fatal error: ', error)

	def _execute(self, query):
		query = json.dumps(query).encode('utf-8')
		result = self._client_execute(None, query)
		if result:
			return json.loads(result.decode('utf-8'))
		
	def _get_chat_id(self, id_n):
		for chat in self.chat_list:
			if chat.whitelist_id == id_n:
				return chat.chat_id
		return False
		
	def send(self, query):
		query = json.dumps(query).encode('utf-8')
		self._client_send(self.client, query)

	def send_message(self, msg):
		if self.selected_chat is None:
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self.selected_chat.chat_id,
				'input_message_content': {
					'@type': 'inputMessageText',
					'text': {
						'@type': 'formattedText',
						'text': msg,
					}
				},
				'@extra': 'sent from Td.py'
			})

	def send_message_multi(self, id_n, msg):
		self.send({
			'@type': 'sendMessage',
			'chat_id': self._get_chat_id(id_n),
			'input_message_content': {
				'@type': 'inputMessageText',
				'text': {
					'@type': 'formattedText',
					'text': msg,
				}
			},
			'@extra': 'sent from Td.py'
		})

	def send_image(self, path):
		if self.selected_chat is None:
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self.selected_chat.chat_id,
				'input_message_content': {
					'@type': 'inputMessagePhoto',
					'photo': {
						'@type': 'inputFileLocal',
						'path': path
					}
				}
			})

	def send_video(self, path):
		if not self.setup_complete():
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self.selected_chat.chat_id,
				'input_message_content': {
					'@type': 'inputMessageVideo',
					'video': {
						'@type': 'inputFileLocal',
						'path': path
					}
				}
			})

	def send_location(self, title, coordinates):
		if not self.setup_complete():
			self.get_chats()
			return False
		else:
			self.send({
				'@type': 'sendMessage',
				'chat_id': self.selected_chat.chat_id,
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

	
	def get_chats(self):
		self.send({
			'@type': 'getChats',
			'limit': 10
		})

	def _get_chat_info(self, chat_id):
		self.send({
			'@type': 'getChat',
			'chat_id': chat_id
		})

	# def select_chat(self, chat):
		# self.selected_chat = chat

	def receive(self):
		result_orig = self._client_receive(self.client, 1.0)
		if result_orig:
			result = json.loads(result_orig.decode('utf-8'))
			recv_type = result['@type']

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
			elif recv_type == 'updateChatOrder':
				if result['chat_id'] not in [chat.chat_id for chat in self.chat_list]:
					self._get_chat_info(result['chat_id'])
			elif recv_type == 'chat':
				if result['type']['@type'] == 'chatTypePrivate':
					self.send({
						'@type': 'getUser',
						'user_id': result['type']['user_id']
					})
				# print(self.command_user_ids)
			elif recv_type == 'user':
				for chat in self.chat_list:
					if chat.phone_number == result["phone_number"]:
						chat.set_chat_id(result["id"])

			return result

	def destroy(self):
		self._client_destroy(self.client)
	
	def setup_complete(self):
		return all([chat.basic_complete for chat in self.chat_list])

class Chat():
	def __init__(self, whitelist_id, phone_number):
		self.chat_id = 0
		self.whitelist_id = whitelist_id
		self.title = ""
		self.phone_number = phone_number

		# type of chat:
		#	0: unknown
		#	1: Private chat (PM)
		#	2: Group
		self.chat_type = 0

		# Only applicable for basic groups (chat type 2 below)
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
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
	def __init__(self, tdlib_dir, output_chat):
	# def __init__(self):
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

		# self.tdlib_dir = rospy.get_param("~tdlib_auth_dir")
		# self.chat_name = rospy.get_param("~output_chat")
		self.tdlib_dir = tdlib_dir
		self.chat_name = output_chat

		self.chat_list = []

		self.selected_chat = None

		self.get_chats()

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
		
	def send(self, query):
		query = json.dumps(query).encode('utf-8')
		self._client_send(self.client, query)
		
	def receive(self):
		result = self._client_receive(self.client, 1.0)
		if result:
			result = json.loads(result.decode('utf-8'))
			recv_type = result['@type']
			# print('================================')
			# print(recv_type)

			if result['@type'] == 'updateAuthorizationState':
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
					new_chat = Chat(result['chat_id'])
					self.chat_list.append(new_chat)
					self._get_chat_info(result['chat_id'])
			elif recv_type == 'chat':
				print([x.chat_id for x in self.chat_list])
				if result['id'] not in [chat.chat_id for chat in self.chat_list]:
					new_chat = Chat(result['id'])
					self.chat_list.append(new_chat)
				
				for chat in self.chat_list:
					if chat.chat_id == result['id']:
						chat.set_title(result['title'])
						chat.set_chat_type(result['type']['@type'])

						if result['title'] == self.chat_name:
							self.selected_chat = chat

			return result

	def destroy(self):
		self._client_destroy(self.client)
	
	def select_chat(self, chat):
		self.selected_chat = chat
	
	def send_message(self, msg):
		if self.selected_chat is None:
			self.get_chats()
			return False
		else:
			# self.selected_chat.send_message(msg)
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
	
	def setup_complete(self):
		return self.selected_chat is not None


class Chat():
	def __init__(self, chat_id):
		self.chat_id = chat_id
		self.title = ""
		self.phone_number = 0

		# type of chat:
		#	0: unknown
		#	1: Private chat (PM)
		self.chat_type = 0

	def set_chat_type(self, chat_type):
		if chat_type == 'chatTypePrivate':
			self.chat_type = 1
	def set_title(self, title):
		self.title = title

	def get_messages(self, n):
		pass

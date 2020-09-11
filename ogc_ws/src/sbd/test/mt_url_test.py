#!/usr/bin/env python3

'''
mt_url_test: Test script to send MT msg to Rockblock through RB server
'''

# Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

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

import binascii
import requests

url = 'https://core.rock7.com/rockblock/MT'
values = {'imei':'', 'username':'', 'password':'', 'data':''}

# Insert msg
msg = 'Hello'
msg = msg.encode()
values['data'] = binascii.hexlify(msg).decode()

# Insert IMEI and credentials
with open('login.txt', 'r') as fp:
    fp.readline() # EC2 Server, we don't need this
    values['imei'] = fp.readline().replace('\n','')
    values['username'] = fp.readline().replace('\n','')
    values['password'] = fp.readline().replace('\n','')

#print(values)

# Send HTTP post request to Rock 7 server
response = requests.post(url, data=values)
print(response.text)
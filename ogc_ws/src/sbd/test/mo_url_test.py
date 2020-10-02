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

import ast
import binascii
import datetime
import requests

starttime = datetime.datetime.utcnow()

with open('login.txt', 'r') as fp:
    url = fp.readline().replace('\n','')

values = {'pw':'testpw'} # We cannot use our account pw, because http is unencrypted...

# Send HTTP post request to Rock 7 server
response = requests.post(url, data=values).text
try:
    data = ast.literal_eval(response) # Convert string to dict
    test_time = datetime.datetime.strptime(data['transmit_time'], "%y-%m-%d %H:%M:%S")
    if test_time < starttime:
        print("No new messages!")
    else:
        print(data['serial'])
        print(data['transmit_time'])
        print(data['data'])
except(ValueError):
    print("Invalid message received!")
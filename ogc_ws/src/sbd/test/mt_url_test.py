#!/usr/bin/env python3

'''
mt_url_test: Test script to send MT msg to Rockblock through RB server
'''

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
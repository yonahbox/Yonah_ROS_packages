#!/usr/bin/env python3

'''
mt_url_test: Test script to send MT msg to Rockblock through RB server
'''

import ast
import binascii
import requests

with open('login.txt', 'r') as fp:
    url = fp.readline().replace('\n','')

values = {'pw':'testpw'} # We cannot use our account pw, because http is unencrypted...

# Send HTTP post request to Rock 7 server
response = requests.post(url, data=values).text
try:
    data = ast.literal_eval(response) # Convert string to dict
    print(data['serial'])
    print(data['data'])
except:
    print("Invalid message received!")
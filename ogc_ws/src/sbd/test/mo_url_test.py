#!/usr/bin/env python3

'''
mo_url_test: Test script to receive a MO msg from Rockblock via web server
'''

# This script should be placed in our web server. To do: Find a way to get the msg from server into GCS

import binascii
import cgi, cgitb

form = cgi.FieldStorage()

# Send respose back to Rock Seven first
print ("Content-Type:text/html\n\n")
print ("OK")

# In the final implementation, should have some form of security checks (e.g. check for whitelisted serial/imei)

# Transfer FieldStorage to a dictionary (for easier manipulation)
params = {}
for key in form.keys():
    params[key] = form[key].value
# "data" field is hex encoded
params["data"] = binascii.unhexlify(params["data"]).decode()

# Write to text file (pre-req: Make sure apache server has rw permissions to the file and its folder)
# See https://www.simplified.guide/apache/change-user-and-group
# and https://stackoverflow.com/questions/33622113/python-cgi-script-permission-denied-when-writing-file
fp = open("/test_folder/mo_test.txt", 'a+')
fp.write(str(params) + "\n")
fp.close()
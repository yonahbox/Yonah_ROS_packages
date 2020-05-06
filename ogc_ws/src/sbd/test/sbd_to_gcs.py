#!/usr/bin/env python3

'''
sbd_to_gcs: Extract MO msg from our web server and sends to GCS (when GCS makes a HTTP post)

Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# This script should be placed in our web server

import binascii
import cgi, cgitb

form = cgi.FieldStorage()
if form["pw"].value == "testpw": # We cannot use our account pw, because http is unencrypted...
    with open("/test_folder/test.txt", 'r') as fp: # Where MO msg is stored. File can only hold 1 msg at a time...
        data = fp.readline().replace("\n","")

# Send data to GCS
print ("Content-Type:text/html\n\n")
print (data)
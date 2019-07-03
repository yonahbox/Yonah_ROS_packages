#!/usr/bin/env python

import time
import socket

while True:
	try:
		client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		client.connect(('localhost', 4000))
		client.send("AIR")
		from_server = client.recv(4096)
		client.close()
		if ("AIR" in from_server) and ("GROUND" in from_server):	
			print "Air-Server-Ground Established"
		
		elif ("AIR" in from_server) and not ("GROUND" in from_server):
			print "Air-Server Established, Server-Ground Connection Down, Please Reconnect"				

	except: 
		print "Air-to-Server Disconnected\n"
		time.sleep(0.5)


	from_server = ''
	time.sleep(0.1)
	

#!/usr/bin/env python

import time
import socket
import subprocess
from subprocess import PIPE

class SSH:

	def __init__(self):
		
		self.__server_address = 'ec2-18-138-24-228.ap-southeast-1.compute.amazonaws.com'
		self.__username = 'ubuntu'
		self.__pkey = 'YonahAWSPrivateKey-8May19.pem'
		self.__ssh_link = False
		self.ssh_linkage = ''

		self.ssh_init()

	def ssh_init(self):
		
		self.ssh_attempt_connection()
		
		if self.ssh_test_connection() == True:
			self.__ssh_link = True			
			#self.netcat_init()


	def ssh_attempt_connection(self):	
		
		print "Attempting connection..."
		self.ssh_linkage = subprocess.Popen(['bash', 'air_ssh_connection.sh'], stdout=PIPE)		

	def ssh_test_connection(self):

		try:
			self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.client.connect(('localhost', 4000))
			self.client.send("AIR")
			self.from_server = self.client.recv(4096)
			self.client.close()
			if ("AIR" in self.from_server) and ("GROUND" in self.from_server):	
				print "Air-Server-Ground Established\n"
		
			elif ("AIR" in self.from_server) and not ("GROUND" in self.from_server):
				print "Air-Server Established, Server-Ground Connection Down, Please Reconnect\n"				

		except: 
			print "Air-to-Server Disconnected\n"


		self.from_server = ''
	
ssh = SSH()

while True:
	ssh.ssh_test_connection()
	time.sleep(0.2)



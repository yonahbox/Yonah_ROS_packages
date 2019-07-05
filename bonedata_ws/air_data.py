#!/usr/bin/env python

import time
import socket
import subprocess
from subprocess import PIPE

class SSH:

	def __init__(self):
		
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage = ''
		self.netcat_linkage = ''
		self.ground_netcat = False

		self.ssh_attempt_connection()

	def ssh_attempt_connection(self):	
		
		print "Attempting connection..."
		self.ssh_linkage = subprocess.Popen(['bash', 'air_ssh_connection.sh'], stdout=PIPE, stderr=PIPE)
		
		time.sleep(5)	

		while self.ssh_test_connection() == False:			
			time.sleep(1)			

		if self.ssh_link == True:
			return True
		else:
			return False
		

	def ssh_test_connection(self):

		try:
			self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.client.connect(('localhost', 4000))
			self.client.send("AIR")
			self.from_server = self.client.recv(4096)
			self.client.close()
			if ("AIR" in self.from_server) and ("GROUND" in self.from_server):	
				print "Air-Server-Ground Established\n"	
				self.ssh_link = True	
			elif ("AIR" in self.from_server) and not ("GROUND" in self.from_server):
				print "Air-Server Established, Server-Ground Connection Down, Please Reconnect\n"
				self.ssh_link = True
				self.netcat_link = False
			if ("NETCAT" in self.from_server):
				self.ground_netcat = True
			else:
				self.ground_netcat = False				

		except: 
			print "Air-Server Disconnected\n"
			self.ssh_link = False
			self.netcat_link = False

		self.from_server = ''

	def netcat_init(self):
	
		
		self.netcat_list = subprocess.Popen(['pidof', 'netcat'], stdout=PIPE).stdout.read()
		self.arg = 'kill -9 ' + self.netcat_list
		subprocess.Popen([self.arg], shell= True)	
		self.netcat_linkage = subprocess.Popen(['bash', 'air_netcat_init.sh'], stdout=PIPE)
		self.netcat_link = True
		print "NETCAT Initialised"

	def ssh_terminate(self):
	
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage.kill()
		self.netcat_linkage.kill()


ssh = SSH()

try:
	while True:
	
		if (ssh.netcat_link == False) and (ssh.ground_netcat == True):
			ssh.netcat_init()
	
		if ssh.ssh_test_connection() == False:
			ssh.ssh_attempt_connection()

		time.sleep(0.2)

except KeyboardInterrupt:
	ssh.ssh_terminate()
	print "Terminating program..."



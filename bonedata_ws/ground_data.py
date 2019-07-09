#!/usr/bin/env python3

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

		self.ssh_attempt_connection()

	def ssh_attempt_connection(self):	
		
		print "Attempting connection...\r\n"
		self.ssh_linkage = subprocess.Popen(['bash', '/home/goodness/Yonah_ROS_packages/bonedata_ws/ground_ssh_connection.sh'], stdout=PIPE, stderr=PIPE, bufsize=-1)
		
		time.sleep(5)	

		while self.ssh_test_connection() == False:			
			time.sleep(1)			

		if self.ssh_link == True:
			self.netcat_init()
			return True
		else:
			return False
		

	def ssh_test_connection(self):

		try:			
			self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.client.connect(('localhost', 4001))
			if self.netcat_link == True:			
				self.client.send("GROUNDNETCAT")
			else:
				self.client.send("GROUND")
			self.from_server = self.client.recv(4096)
			self.client.close()
	
			if ("GROUND" in self.from_server) and ("AIR" in self.from_server):	
				print "Air-Server-Ground Established\r"	
				self.ssh_link = True	
			elif ("GROUND" in self.from_server) and not ("AIR" in self.from_server):
				print "Server-Ground Established, Air-Server Connection Down, Please Reconnect\r"
				self.ssh_link = True				

		except:
			print "Server-Ground Disconnected\r"
			self.ssh_link = False
			self.netcat_link = False

		self.from_server = ''

	def netcat_init(self):
			
		self.netcat_linkage = subprocess.Popen(['bash', '/home/goodness/Yonah_ROS_packages/bonedata_ws/ground_netcat_init.sh'], stdout=PIPE)
		self.netcat_link = True
		print "NETCAT Initialised\r"	

	def ssh_terminate(self):
	
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage.kill()
		self.netcat_linkage.kill()

ssh = SSH()

try:
	while True:

		if ssh.netcat_link == False:
			ssh.netcat_init()
	
		if ssh.ssh_test_connection() == False:
			ssh.ssh_attempt_connection()

		time.sleep(0.2)

except KeyboardInterrupt:
	ssh.ssh_terminate()
	print "Terminating program...\r"



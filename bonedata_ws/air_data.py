#!/usr/bin/env/python

# Imports essential libraries
import os
import time
import subprocess
from subprocess import Popen, PIPE
import rosnode
from std_msgs.msg import String
import rospy
import socket
import multiprocessing


# Creates a ROS node based on air_data.py
#rospy.init_node('air_data', anonymous=False)


# Defines the ROS class and its associated methods
class ROS:

	def __init__(self):
		self.receipt = []
		self.test = None
		to_sms = rospy.Publisher('data_to_sms', String, queue_size=50)
		to_mavros = rospy.Publisher('data_to_mavros', String, queue_size=50)
		from_sms = rospy.Subscriber('data_from_sms', String, self.__subscribe)
		from_mavros = rospy.Subscriber('data_from_mavros', String, self.__subscribe)

		
	def publish(self, dest, message):
		if sms in dest:
			to_sms.publish(message)
		if mavros in dest:
			to_mavros.publish(message)
	
	def __subscribe(self, message):
		self.receipt_log.append(message)

	def return_receipt(self):
		return self.receipt_log

	def clear_receipt(self):
		self.receipt = []
		return True

# Defines the SSH class and its associated methods
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
			self.netcat_init()

	def ssh_test_connection(self):

		try:
			self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.sock.connect(('localhost', 4000))
			self.sock.send("AIR")
			self.from_server = self.recv(4096)
			self.sock.close()
			if ("AIR" in self.from_server) and ("GROUND" in self.from_server):	
				print "Air-Server-Ground Established"
				self.__ssh_link = True
				return True
			elif ("AIR" in self.from_server) and not ("GROUND" in self.from_server):
				print "Air-Server Established, Server-Ground Connection Down, Please Reconnect"	
				self.__ssh_link = False			
				return False
		except: 
			print "Air-to-Server Disconnected\n"
			self.__ssh_link = False
			return False
							
	def ssh_attempt_connection(self):	
		
		print "Attempting connection..."
		self.ssh_linkage = subprocess.Popen(['bash', 'air_ssh_connection.sh'], stdout=PIPE)		

	def ssh_attempt_reconnection(self):

		print "SSH Link Down, Initialising Reconnection Procedure"
		self.ssh_linkage.terminate()
		self.ssh_attempt_connection()
		if (self.ssh_test_connection() == True):
			print "Connection Resumed!"
			self.__ssh_link = True
			self.netcat_linkage.terminate()
			self.netcat_init()
			return True

		else:
			print "Connection Failed"
			self.__ssh_link = False
			return False
	

	def netcat_init(self):
		self.netcat_linkage = subprocess.Popen(['bash', 'air_netcat_init.sh'], stdout=PIPE)
		print "NETCAT Initialised"


	def return_ssh_link():
		return self.__ssh_link
	
			
# Initialising ROS and SSH instances

#ros = ROS()
ssh = SSH()

# Main Program Loop

#while True:
	
#	if (ssh.ssh_test_connection() == False):
#		while (ssh.ssh_attempt_reconnection() == False):
#			time.sleep(2)			
#			continue
	
#	time.sleep(2)
			

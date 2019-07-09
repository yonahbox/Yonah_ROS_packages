#!/usr/bin/env python

import time
import socket
import subprocess
from subprocess import PIPE
import rospy
from std_msgs.msg import String

rospy.init_node('air_data', anonymous=False)

class ROS:

	def __init__(self):
		self.receipt = []
		self.test = None
		to_sms = rospy.Publisher('data_to_sms', String, queue_size=50)
		to_mavros = rospy.Publisher('data_to_mavros', String, queue_size=50)
		from_sms = rospy.Subscriber('data_from_sms', String, self.subscribe)
		from_mavros = rospy.Subscriber('data_from_mavros', String, self.subscribe)

		
	def publish(self, dest, message):
		if sms in dest:
			to_sms.publish(message)
		if mavros in dest:
			to_mavros.publish(message)
	
	def subscribe(self, message):
		self.receipt_log.append(message)

	def return_receipt(self):
		return self.receipt_log

	def clear_receipt(self):
		self.receipt = []
		return True

class SSH:

	def __init__(self):
		
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage = ''
		self.netcat_linkage = ''
		self.ground_netcat = False

		self.ssh_attempt_connection()

	def ssh_attempt_connection(self):	
		
		rospy.loginfo("Attempting connection...")
		print "\r"
		self.ssh_linkage = subprocess.Popen(['bash', '/home/ubuntu/bonedata_ws/src/air_data/src/air_ssh_connection.sh'], stdout=PIPE, stderr=PIPE)
		
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
				rospy.loginfo("Air-Server-Ground Established")
				print "\r"	
				self.ssh_link = True	
			elif ("AIR" in self.from_server) and not ("GROUND" in self.from_server):
				rospy.logwarn("Air-Server Established, Server-Ground Connection Down, Please Reconnect")
				print "\r"
				self.ssh_link = True
				self.netcat_link = False
			if ("NETCAT" in self.from_server):
				self.ground_netcat = True
			else:
				self.ground_netcat = False				

		except: 
			print rospy.logerr("Air-Server Disconnected")
			print "\r"
			self.ssh_link = False
			self.netcat_link = False

		self.from_server = ''

	def netcat_init(self):
		
		self.netcat_list = subprocess.Popen(['pidof', 'netcat'], stdout=PIPE).stdout.read()
		self.arg = 'kill -9 ' + self.netcat_list
		subprocess.Popen([self.arg], shell=True)
		rospy.loginfo("NETCAT Reset")
		print "\r"	
		self.netcat_linkage = subprocess.Popen(['bash', '/home/ubuntu/bonedata_ws/src/air_data/src/air_netcat_init.sh'], stdout=PIPE)
		self.netcat_link = True
		rospy.loginfo("NETCAT Initialised")
		print "\r"

	def ssh_terminate(self):
	
		rospy.loginfo("Program Terminating...")
		print "\r"
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage.kill()
		self.netcat_linkage.kill()


ros = ROS()
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



#!/usr/bin/env/python

# Imports essential libraries
import time
import rospy
import std_msgs.msg import String
import subprocess
from subprocess import Popen, PIPE
import rosnode
import paramiko


# Creates a ROS node based on air_data.py
rospy.init_node('air_data', anonymous=False)


# Defines the ROS class and its associated methods
class ROS:

	def __init__(self):
		self.receipt = []
		self.test = None
		to_sms = rospy.Publisher('data_to_sms', String)
		to_mavros = rospy.Publisher('data_to_mavros', String)
		from_sms = rospy.Subscriber('data_from_sms', String, __subscribe)
		from_mavros = rospy.Subscriber('data_from_mavros', String, __subscribe)

		
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
		self.ssh_attempt_connection()

	def ssh_test_connection(self):
		
		print "Testing Connection..."
		for attempts in range(5):

			probe = subprocess.Popen(['netstat', '-t'], stdout=PIPE)
			network_list = probe.stdout.readlines()
			for i in network_list:
				if ('ec2-18-138-24-228' in i) and ('ESTABLISHED' in i):
					print "SSH Link Connected"
					self.__ssh_link = True
					return True
			time.sleep(0.5)
			continue
		
		print "SSH Not Connected"
		self._ssh_link = False
		return False
							
	
	def ssh_attempt_connection(self):	
		
		print "Attempting connection..."
		self.__ssh = paramiko.SSHClient()		
		self.__ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())			
		try:
			self.__ssh.connect(self.__server_address, username=self.__username, key_filename=self.__pkey, timeout=10)
		except:
			None
		#self.transport = self.__ssh.get_transport()
		#self.transport.open_channel('forwarded-tcpip', (self.__server_address, 4000),('localhost', 4000))
		#print "PORT FORWARDED"

	def ssh_attempt_reconnection(self, counter):

		print "SSH Link Down, Initialising Reconnection Procedure"
		self.__ssh.close()
		self.ssh_attempt_connection()
		if (self.ssh_test_connection() == False):
			if (counter is not 0):						
				self.ssh_attempt_reconnection(counter-1)
				return False			
			else:
				print "Reconnection Procedure Failed"
				return False
		else:
			print "Connection Resumed!"
			return True

	def return_ssh_link():
		return self.__ssh_link
			
# Initialising ROS and SSH instances

ros = ROS()
ssh = SSH()

# Main Program Loop

while True:
	
	if (ssh.ssh_test_connection() == False):
		ssh.ssh_attempt_reconnection(counter=5)
	
	if (ssh.return_ssh_link() == False):
		ros.publish(sms, 'SSH Link Failed')
	
	time.sleep(2)
			

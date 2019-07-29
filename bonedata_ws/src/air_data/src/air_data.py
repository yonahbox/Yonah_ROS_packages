#!/usr/bin/env python

"""
File Name: air_data.py
Date Modified: 26/07/2019
Required Scripts: air_ssh_connection.sh, air_netcat_init.sh

Launched by ROS under air_data.launch, which performs the initialisation of a SSH connection from the companion computer to a web server.

Includes SSH connection, NETCAT initialisation and periodic tests of connection with the web server.

Pending development: interaction with the SMS Link for information exchange.
"""

#Imports critical python modules
import time
import socket
import subprocess
from subprocess import PIPE
import rospy
from std_msgs.msg import String

#Defines the ROS Class that configures the topics that the node subscribes from and publishes to
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

#Defines the SSH Class that handles the network connection between the companion computer and the remote web server
class SSH:

	#Initialises SSH states and attempts connection
	def __init__(self):
		
		self.ssh_link = False
		self.netcat_link = False
		self.ssh_linkage = ''
		self.netcat_linkage = ''
		self.ground_netcat = False

		self.ssh_attempt_connection()

	#Attempts one SSH connection. Waits for 5 seconds before any tests to allow OpenSSH to thoroughly finish the connection process
	def ssh_attempt_connection(self):	
		
		rospy.loginfo("Attempting connection...")		
		print "\r"

		#Usage of python subprocessing to maintain an open SSH connection
		self.ssh_linkage = subprocess.Popen(['bash', '$(find', '-name', '*air_ssh_connection.sh)'], stdout=PIPE, stderr=PIPE)
		
		time.sleep(5)				

		return self.ssh_link		

	#Tests for a valid SSH connection with the web server using sockets
	def ssh_test_connection(self):

		#Attempts to connect to the running socket server on the web server
		try:
			self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.client.connect(('localhost', 4000))
			self.client.send("AIR")
			#Receives a status message from the web server
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

	#Initialises the NETCAT process
	def netcat_init(self):
		
		#Kills any existing NETCAT processes prior to opening a new one, to prevent the hogging of critical ports
		self.netcat_list = subprocess.Popen(['pidof', 'netcat'], stdout=PIPE).stdout.read()
		self.arg = 'kill -9 ' + self.netcat_list
		subprocess.Popen([self.arg], shell=True, stdout=PIPE, stderr=PIPE)
		rospy.loginfo("NETCAT Reset")
		print "\r"
		#Usage of python subprocessing to open a NETCAT process	
		self.netcat_linkage = subprocess.Popen(['bash', '$(find', '-name', '*air_netcat_init.sh)'], stdout=PIPE)
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

if __name__ == "__main__":

	#Creates the air_data node and creates an instance of ROS and SSH
	rospy.init_node('air_data', anonymous=False)
	ros = ROS()
	ssh = SSH()

	try:
		#Loops to ensure that the connection is established, otherwise, the program will continue to attempt connections with the web server until successful
		while True:
	
			if (ssh.netcat_link == False) and (ssh.ground_netcat == True):
				ssh.netcat_init()
	
			if ssh.ssh_test_connection() == False:
				ssh.ssh_attempt_connection()

			time.sleep(1)

	except KeyboardInterrupt:
		ssh.ssh_terminate()



#!/usr/bin/env python3

# Copyright (C) 2019 Seah Shao Xuan, Lau Yan Han, and Yonah (yonahbox@gmail.com)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
File Name: air_data.py
Required Scripts: air_ssh_connection.sh, air_netcat_init.sh
Launched by ROS under air_data.launch, which performs the initialisation of a SSH connection from the companion computer to a web server.
Includes SSH connection, NETCAT initialisation and periodic tests of connection with the web server.
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
		self.sms_status = []
		self.to_sms = rospy.Publisher('data_to_sms', String, queue_size=50)
		self.from_sms = rospy.Subscriber('sms_to_data', String, self.subscribe)
		
	def publish(self, message):
		self.to_sms.publish(message)
	
	def subscribe(self, message):
		self.sms_status.append(message)

	def clear_receipt(self):
		self.sms_status = []

#Defines the SSH Class that handles the network connection between the companion computer and the remote web server
class SSH:

	#Initialises SSH states and attempts connection
	def __init__(self):
		
		self.air_link = False
		self.ground_link = False
		self.netcat_link = False
		self.ssh_linkage = ''
		self.netcat_linkage = ''
		self.ground_netcat = False

		self.ssh_attempt_connection()

	#Attempts one SSH connection. Waits for 5 seconds before any tests to allow OpenSSH to thoroughly finish the connection process
	def ssh_attempt_connection(self):	
		
		rospy.loginfo("Attempting connection...")		
		print ("\r")

		svr_name = rospy.get_param('svr_name')
		svr_ip = rospy.get_param('svr_ip')

		#Usage of python subprocessing to maintain an open SSH connection
		
		port_1 = (10 * rospy.get_param('aircraft')) + 4000
		port_2 = port_1 + 1
		port_3 = port_1 + 2
		self.ssh_linkage = subprocess.Popen(['bash', '/home/ubuntu/Yonah_ROS_packages/bonedata_ws/src/air_data/src/air_ssh_connection.sh', \
			svr_name, svr_ip, str(port_1), str(port_2), str(port_3)], stdout=PIPE, stderr=PIPE)
		
		time.sleep(5)	

		while self.ssh_test_connection('') == False:			
			None			

	#Tests for a valid SSH connection with the web server using sockets	
	def ssh_test_connection(self, sms_status):


		#Attempts to connect to the running socket server on the web server
		try:
			self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.client.connect(('localhost', 4000))
			self.client.send("AIR".encode())
			#Receives a status message from the web server
			self.from_server = self.client.recv(4096)
			self.client.close()
			
			if ("NETCAT".encode() in self.from_server):
				self.ground_netcat = True
			else:
				self.ground_netcat = False

			if ("AIR".encode() in self.from_server) and ("GROUND".encode() in self.from_server):	
				rospy.loginfo("Air-Server-Ground Established")
				print ("\r")	
				self.air_link = True
				self.ground_link = True
				return True	
			elif ("AIR".encode() in self.from_server) and not ("GROUND".encode() in self.from_server):
				rospy.logwarn("Air-Server Established, Server-Ground Connection Down, Please Reconnect")
				print("\r")
				self.air_link = True
				self.ground_link = False
				self.netcat_link = False
				return True
				

		except: 
			rospy.logerr("Air-Server Disconnected")
			print("\r")
			self.air_link = False
			self.ground_link = False
			self.netcat_link = False
			self.ground_netcat = False
			time.sleep(2)
			return False

		self.from_server = ''

	#Initialises the NETCAT process
	def netcat_init(self):
		
		#Kills any existing NETCAT processes prior to opening a new one, to prevent the hogging of critical ports
		self.netcat_list = subprocess.Popen(['pidof', 'netcat'], stdout=PIPE).stdout.read()
		self.arg = 'kill -9 ' + self.netcat_list.decode()
		subprocess.Popen([self.arg], shell=True, stdout=PIPE, stderr=PIPE)
		rospy.loginfo("NETCAT Reset")
		print("\r")
		#Usage of python subprocessing to open a NETCAT process	
		self.netcat_linkage = subprocess.Popen(['bash', '/home/ubuntu/Yonah_ROS_packages/bonedata_ws/src/air_data/src/air_netcat_init.sh'], stdout=PIPE, stderr=PIPE)
		self.netcat_link = True
		rospy.loginfo("NETCAT Initialised")
		print("\r")

	def ssh_terminate(self):
	
		rospy.loginfo("Program Terminating...")
		print("\r")
		self.air_link = False
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

			if ssh.ssh_test_connection(ros.sms_status) == False:
				ssh.ssh_attempt_connection()

			ros.clear_receipt()

			print (ssh.netcat_link)
			print (ssh.ground_netcat)
		
			if (ssh.netcat_link == False) and (ssh.ground_netcat == True):
				ssh.netcat_init()

			# Notify the SMS Tech node about its status
			# @TODO: Receive info about SMS Tech Node's status
			if (ssh.air_link == True) and (ssh.ground_link == True):
				ros.publish("SVC")
			elif (ssh.air_link == True) and (ssh.ground_link == False):
				ros.publish("AIR")
			else:
				ros.publish("DWN")

			time.sleep(1)

	except KeyboardInterrupt:
		ssh.ssh_terminate()



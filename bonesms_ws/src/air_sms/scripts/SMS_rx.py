#!/usr/bin/env python3
"""
SMS_rx: Read an SMS from Ground Control, extract and process the
message inside, and send necessary commands to the aircraft.

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
"""

# Standard Library
import paramiko

# ROS/Third-Party
import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.srv import WaypointPush
from std_msgs.msg import String
from mavros_msgs.msg import Waypoint

# Local
import RuTOS
from waypoint import WP

class SMSrx():
	
	############################
	# Initialization
	############################

	def __init__(self):
		self._username = rospy.get_param("~router_username","root") # Hostname of onboard router
		self._ip = rospy.get_param("~router_ip","192.168.1.1") # IP Adress of onboard router
		self.whitelist = set() # set of whitelisted numbers
		self.msglist = "" # Raw message extracted by router (see https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index)
		self.msg = "" # Actual message sent by GCS. Located on 5th line of msglist

		# Initialize all ROS node and topic communications
		# Make sure all services below are whitelisted in apm_pluginlists.yaml!
		rospy.wait_for_service('mavros/cmd/arming')
		rospy.wait_for_service('mavros/set_mode')
		rospy.wait_for_service('mavros/mission/set_current')
		rospy.wait_for_service('mavros/mission/push')
		self.sms_sender = rospy.Publisher('sendsms', String, queue_size = 5)
		rospy.init_node('SMS_rx', anonymous=False)
		self.rate = rospy.Rate(2)

		# Initialise SSH
		try:
			self.ssh = RuTOS.start_client(self._ip, self._username)
			rospy.loginfo("Connected to router")
		except:
			rospy.logerr("Could not connect to the router")
			raise
		
		# Security and safety measures
		self.populatewhitelist()
		self.purge_residual_sms()

		# Mission-related variables
		self.hop = False
		self.missionlist = []

	def populatewhitelist(self):
		"""Fill up whitelisted numbers. Note that whitelist.txt must be in same folder as this script"""
		textfile = rospy.get_param("~whitelist", "whitelist.txt")
		with open (textfile, "r") as reader:
			for line in reader:
				if line[-1] == "\n":
					self.whitelist.add(line[:-1]) # remove whitespace at end of line
				else:
					self.whitelist.add(line) # last line
		rospy.loginfo(self.whitelist)

	def purge_residual_sms(self):
		"""
		Clear the router of SMSes (capped at 30 for the RUT955) on bootup. This prevents the situation
		where an SMS is received when the air router was off, and is acted upon the moment the router
		switches on (very dangerous if that SMS is an arm msg!)
		"""
		# The bad thing about this implementation is that it increases boot time by 30 seconds
		count = 1
		rospy.loginfo("Purging residual SMS, please wait...")
		while count <= 30:
			try:
				RuTOS.delete_msg(self.ssh, count)
				count += 1
			except:
				count += 1
				continue
		rospy.loginfo("Purge complete!")

	#####################################
	# Interaction with ROS Services/Nodes
	#####################################

	def log_and_ack_msg(self):
		'''Log msg and pass it to SMS_tx (through sms_sender topic) for acknowledgement purposes'''
		rospy.loginfo(self.msg)
		self.sms_sender.publish(self.msg)
	
	def checkArming(self):
		"""Check for Arm/Disarm commands from Ground Control"""
		arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		if self.msg == "disarm":
			arm(0)
		elif self.msg == "arm":
			arm(1)
		else:
			return
		self.log_and_ack_msg()

	def checkMode(self):
		"""Check for Mode change commands from Ground Control"""
		mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		# Message structure: mode <flight mode>; extract 2nd word to get flightmode
		if 'mode' in self.msg:
			mode_breakdown = self.msg.split()
			if mode_breakdown[0] == 'mode' and len(mode_breakdown) == 2:
				# Set flight mode, check if successful
				if mode(custom_mode = mode_breakdown[1]).mode_sent == True:
					self.log_and_ack_msg()
	
	def checkSMS(self):
		"""
		Handle all services related to sending of SMS to Ground Control
		If SMS sending is required, instruct SMS_tx (through the sendsms topic) to do it
		"""
		prefixes = ["sms", "ping", "statustext"]
		for i in prefixes:
			if i in self.msg:
				breakdown = self.msg.split()
				# Commands are "sms/ping/statustext <command>"" or "ping"
				if breakdown[0] == i and len(breakdown) <= 2:
					self.log_and_ack_msg()
				break

	def checkMission(self):
		# wpfolder = rospy.get_param('~waypoint_folder', '/home/ubuntu/Yonah_ROS_packages/Waypoints/')
		wpfolder = "/home/huachen/Yonah/Yonah_ROS_packages/Waypoints/"
		"""Check for mission/waypoint commands from Ground Control"""
		if "wp" in self.msg:
			wp_breakdown = self.msg.split()
			if wp_breakdown[0] == "wp" and len(wp_breakdown) == 3:
				if wp_breakdown[1] == 'set':
					# Message structure: wp set <seq_no>, extract the 3rd word to get seq no
					wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
					seq_no = wp_breakdown[2]
					# Set target waypoint, check if successful
					if wp_set(wp_seq = int(seq_no)).success == True:
						self.log_and_ack_msg()
				elif wp_breakdown[1] == 'load':            
					# Message structure: wp load <wp file name.txt>; extract 3rd word to get wp file
					# Assume that wp file is located in root directory of Beaglebone
					wp_file = wp_breakdown[2]
					# Set to non-hop mission
					self.hop = False
					# Reset mission and waypoints list
					self.missionlist = []
					readwp = WP()
					try:
						waypoints = readwp.read(str(wpfolder + wp_file))
						wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
						# Push waypoints, check if successful
						if wp(0, waypoints).success:
							self.log_and_ack_msg()
					except FileNotFoundError:
						print("Specified file not found")
					except:
						print("Invalid waypoint file")
						raise
				else:
					return
		if "mission" in self.msg:
			mission_breakdown = self.msg.split()
			if mission_breakdown[1] == 'load':
				# Message structure: mission load <mission file name.txt>
				self.missionlist = []
				# Change to hop-mission mode
				self.hop = True
				mission_file = mission_breakdown[2]
				try:
					f = open(str(wpfolder + mission_file), "r")
				except FileNotFoundError:
					print("Specified file not found")
					return
				for line in f:
					# Ignores # comments
					if line.startswith('#'):
						continue
					# Returns if a waypoint file is loaded instead
					elif line.startswith("QGC WPL"):
						print("This is a waypoint file. Please load a mission file.")
						return
					try:
						g = open(str(wpfolder + line.rstrip()), "r") # Open and close to check each wp file
						g.close()
					except FileNotFoundError:
						# Specify which file in the list is not found
						print(str(line.rstrip() + "-->File not found"))
						# Set to non-hop (in this case it is used as a switch for the next step)
						self.hop = False
				f.close()
				# Returns if any of the files in mission list weren't found
				if not self.hop:
					return
				# Somehow there is a need to open the file again after the try block finishes
				f = open(str(wpfolder + mission_file), "r")
				# This appends the waypoint files into mission list
				for line in f:
					if line.startswith('#'):
						continue
					self.missionlist.append(line.rstrip())
				f.close()
				# Prints the missions for operator to check
				print("Missions: " + ", ".join(self.missionlist))
				# Load first mission
				self.current_mission = 0
				readwp = WP()
				waypoints = readwp.read(str(wpfolder + self.missionlist[0]))
				wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
				if wp(0, waypoints).success:
					# This acknowledgement implies that the mission list has no errors and the first mission is loaded
					self.log_and_ack_msg()

			elif mission_breakdown[1] == 'next':
				# Message structure: wp next (no arguments)
				if not self.hop: # Checks if hop mission
					print("Please load a mission file")
					return
				self.current_mission += 1
				if self.current_mission >= len(self.missionlist):
					self.hop = False
					self.current_mission = 0
					print("There are no more missions")
					return
				waypoints = []
				readwp = WP()
				try:
					waypoints = readwp.read(str(wpfolder + self.missionlist[self.current_mission]))
					wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
					if wp(0, waypoints).success:
						self.log_and_ack_msg()
				# This error should never be raised if everything above works
				except FileNotFoundError:
					print("Specified file not found")
			else:
				return
	
	############################
	# "Main" function
	############################
	
	def client(self):
		"""Main function to let aircraft receive SMS commands"""
		while not rospy.is_shutdown():
			try:
				# Read an SMS received by the air router
				self.msglist = RuTOS.extract_msg(self.ssh, 1)
				if 'no message\n' in self.msglist:
					pass
				elif 'N/A\n' in self.msglist:
					pass
				elif 'Timeout.\n' in self.msglist:
					rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
				elif 'Connection lost' in self.msglist:
					rospy.logerr("Connection to router lost!")
				else:
					# extract sender number (2nd word of 3rd line in msglist)
					sender = self.msglist[2].split()[1]
					# Ensure sender is whitelisted before extracting message
					if sender in self.whitelist:
						rospy.loginfo('Command from '+ sender)
						# msg is located on the 5th line (minus first word) of msglist. It is converted to lowercase
						self.msg = (self.msglist[4].split(' ', 1)[1].rstrip()).lower()
						# Run through a series of checks to see what command should be sent to aircraft
						self.checkArming()
						self.checkSMS()
						self.checkMode()
						self.checkMission()
					else:
						rospy.logwarn('Rejected msg from unknown sender ' + sender)
					RuTOS.delete_msg(self.ssh, 1) # Delete the existing SMS
			
			except(rospy.ServiceException):
				rospy.logwarn("Service call failed")
			
			self.rate.sleep()

if __name__=='__main__':
	try:
		run = SMSrx()
		run.client()
	except:
		rospy.loginfo("Failed to start node")
		raise
	else:
		run.ssh.close()
		rospy.loginfo("Connection to router closed")
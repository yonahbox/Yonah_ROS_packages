#!/usr/bin/env python3

'''
g2a: Module to handle ground-to-air commands
'''

# Copyright (C) 2020, Wang Huachen and Yonah (yonahbox@gmail.com)

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

import waypoint
import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent

recognised_commands = ["ping", "sms", "statustext", "arm", "disarm", "mode", "wp", "mission", "syncthing"]

def check_ping(self):
	'''Check for ping commands from Ground Control'''
	if len(self._recv_msg) == 1: # Simple "ping" request
		self._msg = self.payloads.truncate_regular_payload()
		severity = "r"
	else:
		# Make sure the ping command is of the correct format ("ping <command>")
		if len(self._recv_msg) == 2 and self._recv_msg[0] == 'ping' and \
			self._recv_msg[1] in self.payloads.ping_entries:
			self._msg = str(self.payloads.ping_entries[self._recv_msg[1]])
			severity = "i"
		else:
			return
	self.sendmsg(severity)

def check_sms(self, uuid):
	'''Check for SMS commands from Ground Control'''
	if self._recv_msg[0] == "sms" and len(self._recv_msg) == 2:
		if self._recv_msg[1] == "true": # Send regular payloads to Ground Control
			self._regular_payload_flag = True
		elif self._recv_msg[1] == "false": # Don't send regular payloads
			self._regular_payload_flag = False
		elif self._recv_msg[1] == "short": # Send regular payloads at short intervals
			self._sms_interval = self._interval_1
		elif self._recv_msg[1] == "long": # Send regular payloads at long intervals
			self._sms_interval = self._interval_2
		else:
			return
	else:
		return
	self._send_ack(uuid)

def check_statustext(self, uuid):
	'''Check for statustext commands from Ground Control'''
	if len(self._recv_msg) == 2 and self._recv_msg[0] == "statustext":
		if self._recv_msg[1] == "true":
			self._statustext_flag = True
		elif self._recv_msg[1] == "false":
			self._statustext_flag = False
		else:
			return
	else:
		return
	self._send_ack(uuid)

def check_arming(self, uuid):
	"""Check for Arm/Disarm commands from Ground Control"""
	rospy.logwarn("G2A: Trying to arm")
	arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	if len(self._recv_msg) == 1:
		if self._recv_msg[0] == "disarm":
			arm(0)
		elif self._recv_msg[0] == "arm":
			arm(1)
			self.syncthing_control.publish("pause")
		else:
			return
	else:
		return
	self._send_ack(uuid)

def check_mode(self, uuid):
	"""Check for Mode change commands from Ground Control"""
	mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
	# Message structure: mode <flight mode>; extract 2nd word to get flightmode
	if self._recv_msg[0] == 'mode' and len(self._recv_msg) == 2:
		# Set flight mode, check if successful
		if mode(custom_mode = self._recv_msg[1]).mode_sent == True:
			self._send_ack(uuid)

def check_mission(self, uuid):
	"""Check for mission/waypoint commands from Ground Control"""
	if self._recv_msg[0] == "wp":
		if self._recv_msg[1] == 'set':
			wp_set(self, uuid)
		elif self._recv_msg[1] == 'load':
			wp_load(self, uuid)
		else:
			return

	elif self._recv_msg[0] == "mission":
		if self._recv_msg[1] == 'load':
			mission_load(self)
		elif self._recv_msg[1] == 'next':
			mission_next(self)
		elif self._recv_msg[1] == 'update':
			mission_update(self)
		else:
			return

def wp_set(self, uuid):
	# Message structure: wp set <seq_no>, extract the 3rd word to get seq no
	wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
	seq_no = self._recv_msg[2]
	# Set target waypoint, check if successful
	if wp_set(wp_seq = int(seq_no)).success == True:
		self._send_ack(uuid)

def wp_load(self, uuid):
	# Message structure: wp load <wp file name.txt>; extract 3rd word to get wp file
	# Assume that wp file is located in Waypoints folder of Beaglebone
	wp_file = self._recv_msg[2]
	# Set to non-hop mission
	self.hop = False
	self.pub_to_rff.publish("hop False")
	# Reset mission and waypoints list
	self.missionlist = []
	readwp = waypoint.WP()
	try:
		waypoints = readwp.read(str(self.wpfolder + wp_file))
		waypoint.push_waypoints(self, waypoints)
		self._send_ack(uuid)
	except FileNotFoundError:
		self._msg  = "Specified file not found"
		self.sendmsg("e")
	except:
		self._msg = "Invalid waypoint file"
		self.sendmsg("e")

def mission_load(self):
	# Message structure: mission load <mission file name.txt>
	self.missionlist = []
	mission_file = self._recv_msg[2]
	try:
		f = open(str(self.wpfolder + mission_file), "r")
	except FileNotFoundError:
		self._msg = "Specified file not found"
		self.sendmsg("e")
		return
	for line in f:
		# Ignores # comments
		if line.startswith('#'):
			continue
		# Returns if a waypoint file is loaded instead
		elif line.startswith("QGC WPL"):
			self._msg = "This is a waypoint file. Please load a mission file."
			self.sendmsg("e")
			return
		try:
			g = open(str(self.wpfolder + line.rstrip()), "r") # Open and close to check each wp file
			g.close()
		except FileNotFoundError:
			# Specify which file in the list is not found
			self._msg = str(line.rstrip() + "-->File not found")
			self.sendmsg("e")
		else:
			# Change to hop-mission mode
			self.hop = True
			self.pub_to_rff.publish("hop True")
	f.close()
	# Returns if any of the files in mission list weren't found
	if not self.hop:
		return
	# Somehow there is a need to open the file again after the try block finishes
	f = open(str(self.wpfolder + mission_file), "r")
	# This appends the waypoint files into mission list
	for line in f:
		if line.startswith('#'):
			continue
		self.missionlist.append(line.rstrip())
	f.close()
	# Prints the missions for operator to check
	self._msg = "Missions: " + ", ".join(self.missionlist)
	self.sendmsg("i")
	# Load first mission
	self.current_mission = -1
	#CLEAR WAYPOINTS

def mission_next(self):
	# Message structure: wp next (no arguments)
	if self.payloads.entries["arm"] == True:
		self._msg = "Plane is armed. Unable to load next mission"
		self.sendmsg("e")
		return
	if not self.hop: # Checks if hop mission
		self._msg = "Please load a mission file"
		self.sendmsg("e")
		return
	self.current_mission += 1
	if self.current_mission >= len(self.missionlist):
		self.hop = False
		self.pub_to_rff.publish("hop False")
		self.current_mission = -1
		self._msg = "There are no more missions"
		self.sendmsg("e")
		return
	waypoints = []
	readwp = waypoint.WP()
	try:
		waypoints = readwp.read(str(self.wpfolder + self.missionlist[self.current_mission]))
		waypoint.push_waypoints(self, waypoints)
		self.pub_to_rff.publish("ack")
		self._msg = "Loaded next mission: " + str(self.missionlist[self.current_mission])
		self.sendmsg("i")
	# This error should never be raised if everything above works
	except FileNotFoundError:
		self._msg = "Specified file not found"
		self.sendmsg("e")

def mission_update(self):
	gndtime = {}
	for i in range(2, len(self._recv_msg)):
		if i % 2 == 0:
			gndtime[self._recv_msg[i]] = int(self._recv_msg[i+1])
	airtime = waypoint.get_update_time(self.wpfolder)
	required_files = waypoint.compare_time(gndtime, airtime)
	self._msg = " ".join(required_files)
	self.sendmsg("m")

def handle_syncthing(self, uuid):
	if self._recv_msg[0] == 'syncthing' and len(self._recv_msg) == 2:
		if self._recv_msg[1] in ["pause", "resume"]:
			self.syncthing_control.publish(self._recv_msg[1])
			self._send_ack(uuid)
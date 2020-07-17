#!/usr/bin/env python3

import waypoint
import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from missionserver import MissionServer


# Wait for MAVROS services
# rospy.wait_for_service('mavros/cmd/arming')
# rospy.wait_for_service('mavros/set_mode')
# rospy.wait_for_service('mavros/mission/set_current')

recognised_commands = ["ping", "sms", "statustext", "arm", "disarm", "mode", "wp", "mission"]

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

def check_sms(self):
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
	self._send_ack()

def check_statustext(self):
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
	self._send_ack()

def check_arming(self):
	"""Check for Arm/Disarm commands from Ground Control"""
	arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	if len(self._recv_msg) == 1:
		if self._recv_msg[0] == "disarm":
			arm(0)
		elif self._recv_msg[0] == "arm":
			arm(1)
		else:
			return
	else:
		return
	self._send_ack()

def check_mode(self):
	"""Check for Mode change commands from Ground Control"""
	mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
	# Message structure: mode <flight mode>; extract 2nd word to get flightmode
	if self._recv_msg[0] == 'mode' and len(self._recv_msg) == 2:
		# Set flight mode, check if successful
		if mode(custom_mode = self._recv_msg[1]).mode_sent == True:
			self._send_ack()

def check_mission(self):
	"""Check for mission/waypoint commands from Ground Control"""
	if self._recv_msg[0] == "wp":
		if self._recv_msg[1] == 'set':
			wp_set(self)
		elif self._recv_msg[1] == 'load':
			wp_load(self)
		else:
			return

	elif self._recv_msg[0] == "mission":
		if self._recv_msg[1] == 'load':
			mission_load(self)
		elif self._recv_msg[1] == 'next':
			mission_next(self)
		elif self._recv_msg[1] == 'write':
			mission_write(self)
		elif self._recv_msg[1] == 'update':
			mission_update(self)
		elif self._recv_msg[1] == 'server':
			mission_server_update(self)
		else:
			return

def wp_set(self):
	# Message structure: wp set <seq_no>, extract the 3rd word to get seq no
	wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
	seq_no = self._recv_msg[2]
	# Set target waypoint, check if successful
	if wp_set(wp_seq = int(seq_no)).success == True:
		self._send_ack()

def wp_load(self):
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
	except FileNotFoundError:
		self._msg  = "Specified file not found"
		self.sendmsg("e")
	except:
		raise
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
		elif line.startswith('Last update'):
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
		elif line.startswith('Last update'):
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
	# This error should never be raised if everything above works
	except FileNotFoundError:
		self._msg = "Specified file not found"
		self.sendmsg("e")

def mission_write(self):
	f = open(self.wpfolder + "missionlist.txt", "w")
	for i in self._recv_msg[2:]:
		f.write(i + "\n")
	f.write("Last update: " + str(rospy.get_rostime().secs) + "\n")
	f.close()

def mission_update(self):
	gndtime = {}
	for i in range(2, len(self._recv_msg)):
		if i % 2 == 0:
			gndtime[self._recv_msg[i]] = int(self._recv_msg[i+1])
	airtime = waypoint.get_update_time(self.wpfolder)
	required_files = waypoint.compare_time(gndtime, airtime)
	self._msg = " ".join(required_files)
	self.sendmsg("m")

def mission_server_update(self):
	run = MissionServer()
	run.update_files()
	run.close()
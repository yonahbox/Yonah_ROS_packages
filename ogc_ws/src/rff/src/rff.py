#!/usr/bin/env python3

import time
import rospy
import RuTOS
import csv
import threading
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import State
from pathlib import Path
from waypoint import WP

missionlist = []

class RFF:

	def __init__(self):
		# Check arm status of aircraft
		rospy.Subscriber('/mavros/state', State, self.checkDisarm)
		# Path to missionlist.txt which contains all missions in a flight in order
		path = str(waypointsfolder + "mission_list.txt")
		f = open(path, "r")
		for line in f:
			# Ignores # comments
			if line.startswith('#'):
				continue
			if line.startswith('Last update:'):
				continue
			missionlist.append(line.rstrip())
		# Prints the missions for operator to check
		rospy.loginfo("Missions for this flight:")
		for i in missionlist:
			rospy.loginfo(i)
		# Assumes armed for safety
		self.armStatus = True

	def main(self):
		rospy.loginfo("Loading waypoints.")
		rospy.wait_for_service('mavros/mission/push', timeout=10)
		wppush = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
		while not wppush(0, waypoints).success:
			rospy.logerr("Failed to load waypoints. Trying again.")
			time.sleep(3)
		rospy.loginfo("Waypoints loaded.")
		rospy.wait_for_service('mavros/mission/set_current', timeout=10)
		wpset = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
		while not wpset(1).success:
			rospy.logerr("Failed to set current waypoint. Trying again.")
			time.sleep(3)
		rospy.wait_for_service('mavros/set_mode', timeout=10)
		mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		while not mode(custom_mode = "AUTO").mode_sent:
			rospy.logerr("Failed to set mode to AUTO. Trying again.")
			time.sleep(3)
		rospy.loginfo("MODE = AUTO")
		rospy.loginfo("Arming in 10 seconds. Please stand clear.")
		time.sleep(10)
		rospy.wait_for_service('mavros/cmd/arming', timeout=10)
		arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		while not arm(1).success:
			rospy.logwarn("Failed to arm throttle. Trying again. Please continue to stay clear of aircraft.")
			time.sleep(3)
		rospy.loginfo("THROTTLE ARMED")
		self.armStatus = True

	def checkDisarm(self, data):
		self.armStatus = data.armed

class Button:

	def __init__(self):
		self._username = rospy.get_param("~router_username","root") # Hostname of onboard router
		self._ip = rospy.get_param("~router_ip","192.168.1.1") # IP Adress of onboard router
		self.Button_State = False
		self.stopwarn = False
		self.ssh = RuTOS.start_client(self._ip, self._username)
		rospy.loginfo("Monitoring button")
		RuTOS.button_on(self.ssh)

	def blink(self):
		while True:
			RuTOS.blink_button(self.ssh)
			time.sleep(0.5)
			if self.stopwarn:
				break

	def press(self):
		while True:
			digital_input = RuTOS.check_button(self.ssh)

			if digital_input == 0:
				if self.Button_State == False:
					timedown = time.time()
					rospy.loginfo("Button pressed.")
				self.Button_State = True

			elif digital_input == 1:
				if self.Button_State == True:
					timeup = time.time()
					timeheld = timeup - timedown
					if timeheld >= 2:
						blinker = threading.Thread(target = self.blink)
						blinker.start()
						rff.main()
						self.stopwarn = True
						blinker.join()
						break
					rospy.loginfo("Button not held for at least 2 seconds. Continuing to monitor.")
				self.Button_State = False

			time.sleep(0.5)

		self.ssh.close()
		rospy.loginfo("Disconnected from button.")

if __name__ == "__main__":
	rospy.init_node('rff', anonymous=False, disable_signals=True)
	waypointsfolder = rospy.get_param('~waypoint_folder', '/home/ubuntu/Waypoints/')
	rff = RFF()

	try:
		for i in range(len(missionlist)):
			wpfile = str(waypointsfolder + missionlist[i])
			wpread = WP()
			waypoints = wpread.read(wpfile)
			# Do nothing if aircraft is armed
			while rff.armStatus:
				time.sleep(5)
			rospy.loginfo("Plane has been disarmed. Starting button monitoring. ")
			button = Button()
			button.press()
			rospy.loginfo(missionlist[i] + " started.")
			waypoints = []
		
		rospy.loginfo("All missions finished")

	except KeyboardInterrupt:
		rospy.signal_shutdown("Shutting down")

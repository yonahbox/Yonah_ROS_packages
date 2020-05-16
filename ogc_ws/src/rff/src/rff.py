#!/usr/bin/env python3

import time
import rospy
import csv
from pathlib import Path
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import Waypoint

missionlist = []
waypoints = []
waitforbutton = True

class WP(object):

	class CSVDialect(csv.Dialect):
		delimiter = '\t'
		doublequote = False
		skipinitialspace = True
		lineterminator = '\r\n'
		quoting = csv.QUOTE_NONE

	def read(self, wpfile):
		f = open(wpfile, "r")
		pastheaderline = False
		for data in csv.reader(f, self.CSVDialect):
			if not pastheaderline:
				qgc, wpl, ver = data[0].split(' ', 3)
				ver = int(ver)
				if qgc == 'QGC' and wpl == 'WPL' and (ver == 110 or ver ==120):
					pastheaderline = True

			else:
				waypoints.append(Waypoint(
					is_current = bool(int(data[1])),
					frame = int(data[2]),
					command = int(data[3]),
					param1 = float(data[4]),
					param2 = float(data[5]),
					param3 = float(data[6]),
					param4 = float(data[7]),
					x_lat = float(data[8]),
					y_long = float(data[9]),
					z_alt = float(data[10]),
					autocontinue = bool(int(data[11]))
				))


class RFF:

	def __init__(self):
		rospy.Subscriber('buttonpress', String, self.subscribe)
		self.start_return_flight = False
		f = open("missionlist.txt", "r")
		for line in f:
			missionlist.append(line.rstrip())
		rospy.loginfo("Missions:")
		for i in missionlist:
			rospy.loginfo(i)

	def subscribe(self, state):
		self.start_return_flight = state.data
		if state.data == 'True':
			self.start_return_flight = True
		else:
			self.start_return_flight = False
		self.main()

	def main(self):
		global waitforbutton
		if self.start_return_flight == True:
			rospy.loginfo("Button pressed.")
			# rospy.wait_for_service('mavros/mission/push')
			wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
			wp(0, waypoints)
			rospy.loginfo("Waypoints loaded")
			# rospy.wait_for_service('mavros/mission/set_current')
			wpset = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
			wpset(1)
			# rospy.wait_for_service('mavros/set_mode')
			mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
			mode(custom_mode = "AUTO")
			rospy.loginfo("MODE = AUTO")
			rospy.loginfo("Arming in 10 seconds")
			time.sleep(10)
			# rospy.wait_for_service('mavros/cmd/arming')
			arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
			arm(1)
			rospy.loginfo("THROTTLE ARMED")
			time.sleep(1)
			waitforbutton = False


if __name__ == "__main__":
	rospy.init_node('rff', anonymous=False, disable_signals=True)
	rff = RFF()

	try:
		for i in range(len(missionlist)):
			wpfile = missionlist[i]
			readwp = WP()
			readwp.read(wpfile)
			while waitforbutton == True:
				rff.main()
				time.sleep(1)
			rospy.loginfo("Mission started. Loading next mission.")
			waypoints = []
			waitforbutton = True
		
		print("All missions finished")

	except KeyboardInterrupt:
		pass
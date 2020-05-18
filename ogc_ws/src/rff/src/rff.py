#!/usr/bin/env python3

import time
import rospy
import paramiko
import csv
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import State

missionlist = []
waypoints = []
waypointsfolder = "/home/Waypoints/"

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
		rospy.Subscriber('/mavros/state', State, self.checkDisarm)
		path = str(waypointsfolder + "missionlist.txt")
		f = open(path, "r")
		for line in f:
			missionlist.append(line.rstrip())
		rospy.loginfo("Missions for this flight:")
		for i in missionlist:
			rospy.loginfo(i)
		# Assumes armed
		self.armStatus = True

	def main(self):
		rospy.loginfo("Loading waypoints.")
		# rospy.wait_for_service('mavros/mission/push')
		wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
		wp(0, waypoints)
		rospy.loginfo("Waypoints loaded.")
		# rospy.wait_for_service('mavros/mission/set_current')
		wpset = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
		wpset(1)
		# rospy.wait_for_service('mavros/set_mode')
		mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		mode(custom_mode = "AUTO")
		rospy.loginfo("MODE = AUTO")
		rospy.loginfo("Arming in 10 seconds. Please stand clear.")
		time.sleep(10)
		# rospy.wait_for_service('mavros/cmd/arming')
		arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		arm(1)
		rospy.loginfo("THROTTLE ARMED")
		time.sleep(1)

	def checkDisarm(self, data):
		self.armStatus = data.armed

class Button:

	def __init__(self):
		self.Button_State = False
		self.ssh = paramiko.SSHClient()
		self.ssh.load_system_host_keys()
		self.ssh.connect('192.168.1.1', username='root')
		rospy.loginfo("Monitoring button")

	def press(self):
		while True:
			din, dout, derr = self.ssh.exec_command('gpio.sh get DIN1')
			digital_input = int(str(dout.readlines())[2])

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
						rff.main()
						break
					rospy.loginfo("Button not held for at least 2 seconds. Continuing to monitor.")
				self.Button_State = False

			time.sleep(0.5)

if __name__ == "__main__":
	rospy.init_node('rff', anonymous=False, disable_signals=True)
	rff = RFF()

	try:
		for i in range(len(missionlist)):
			wpfile = str(waypointsfolder + missionlist[i])
			readwp = WP()
			readwp.read(wpfile)
			while rff.armStatus:
				time.sleep(5)
			rospy.loginfo("Plane has been disarmed. Starting button monitoring. ")
			button = Button()
			button.press()
			rospy.loginfo(missionlist[i] + " started.")
			waypoints = []
		
		print("All missions finished")

	except KeyboardInterrupt:
		pass
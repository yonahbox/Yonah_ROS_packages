#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText

typetable = {
	0 : "Throttle", 
	1 : "Mission", 
	2 : "Waypoint", 
	3 : "Transition", 
	4 : "Land", 
	5 : "Prearm", 
	6 : "", 
	7 : "", 
	8 : "", 
	9 : "", 

}

navcommandtable = {
	16 : "Waypoint", 
	17 : "Loiter Unlimited", 
	18 : "Loiter Turns", 
	19 : "Loiter Time", 
	20 : "Return to Launch", 
	21 : "Land", 
	22 : "Takeoff", 
	30 : "Continue Change Alt", 
	31 : "Loiter to Alt", 
	83 : "Altitude Wait", 
	84 : "VTOL Takeoff", 
	85 : "VTOL Land", 
	112 : "Delay", 
	113 : "Distance", 
	176 : "Set Mode", 
	177 : "Jump", 
	178 : "Change Speed", 
	179 : "Set Home", 
	181 : "Set Relay", 
	182 : "Repeat Relay", 
	183 : "Set Servo", 
	184 : "Repeat Servo", 
	189 : "Land Start", 
	200 : "Control Video", 
	201 : "Set ROI", 
	202 : "Digicam Configure", 
	205 : "Mount Control", 
	206 : "Set Cam Trigger Distance", 
	207 : "Fence Enable", 
	208 : "Parachute", 
	210 : "Inverted Flight", 
	212 : "Autotune Enable", 
	223 : "Engine Control", 
	3000 : "VTOL Transition", 
}

class Handler:

	def __init__(self):
		self.msg = ""
		self.prefix = ""
		rospy.init_node('statustext', anonymous=True)
		rospy.Subscriber("/mavros/statustext/recv/", StatusText, self.callback)
		rospy.spin()

	def callback(self, data):
		if data.severity >= 6:
			self.prefix = "I"
			if data.text.startswith("Throttle"):
				self.type = 0
				self.throttletext(data.text)
			elif data.text.startswith("Executing"):
				self.type = 1
				self.missiontext(data.text)
			# Plane 4.0
			# elif data.text.startswith("Mission"):
			# 	self.type = 1
			# 	self.missiontext4(data.text)
			elif data.text.startswith("Reached"):
				self.type = 2
				self.waypointtext(data.text)
			elif data.text.startswith("Transition"):
				self.type = 3
				self.transitiontext(data.text)
			elif data.text.startswith("Land"):
				self.type = 4
				self.landtext(data.text)
		elif data.severity >= 4:
			self.prefix = "W"
			# print(data.text)
		else:
			self.prefix = "E"
			if data.text.startswith("PreArm"):
				self.type = 5
				print("Cannot arm at the moment")
			else:
				print("This is emergency " + str(data.text))

	def throttletext(self,text):
		self.throttlestatus = text.split()[1]
		print("Throttle " + str(self.throttlestatus))

	def missiontext(self, text):
		self.currentmission = int(text.split()[-1][1:])
		print("Current mission: " + navcommandtable.get(self.currentmission))
		
	# Plane 4.0
	# def missiontext4(self, text):
	# 	For plane 4.0
	# 	self.missionitem = text.split()[1]
	# 	self.missiondefine = text.split()[2]
	# 	print("Starting mission item " + str(self.missionitem) + ": " + str(self.missiondefine))
		
	def waypointtext(self, text):
		print("Reached waypoint " + str(text.split()[2][1:]))

	def transitiontext(self, text):
		if len(text) > 15:
			self.transitionspeed = text.split()[3]
			print("Transitioning speed " + self.transitionspeed + "m/s")
		else:
			print("Transition done")

	def landtext(self,text):
		if text.split()[1] == "descend":
			print("Land descend started")
		elif text.split()[1] == "final":
			print("Land final started")
		else:
			print("Land complete")


if __name__ == '__main__':
	
	try:
		statustexthandler = Handler()

	except rospy.ROSInterruptException:
		pass
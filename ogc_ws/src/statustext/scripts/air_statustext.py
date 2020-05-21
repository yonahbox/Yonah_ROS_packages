#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText

typetable = {
	0 : "Throttle",
	1 : "Mission",
	2 : "Transition",
	3 : "Land",
	4 : "Prearm",
	5 : "",
	6 : "",
	7 : "",
	8 : "",
	9 : "",

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
			elif data.text.startswith("Transition"):
				self.type = 2
				self.transitiontext(data.text)
			elif data.text.startswith("Land"):
				self.type = 3
				self.landtext(data.text)
		elif data.severity >= 4:
			self.prefix = "W"
			# print(data.text)
		else:
			self.prefix = "E"
			if data.text.startswith("PreArm"):
				self.type = 4
				print("Cannot arm at the moment")
			else:
				print("This is emergency " + str(data.text))

	def throttletext(self,text):
		self.throttlestatus = text.split()[1]
		print("Throttle " + str(self.throttlestatus))

	def missiontext(self, text):
		self.currentmission = text.split()[-1]
		# Look up in command ID table
		pass

	# Plane 4.0
	# def missiontext4(self, text):
	# 	For plane 4.0
	# 	self.missionitem = text.split()[1]
	# 	self.missiondefine = text.split()[2]
	# 	print("Starting mission item " + str(self.missionitem) + ": " + str(self.missiondefine))
		
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
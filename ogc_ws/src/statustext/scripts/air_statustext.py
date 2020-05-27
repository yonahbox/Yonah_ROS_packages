#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText
from statustext.msg import YonahStatusText


class StatusTextHandler:

	def __init__(self):
		rospy.init_node('air_statustext', anonymous=False)
		self.pub = rospy.Publisher('ogc/statustext', YonahStatusText, queue_size=5)
				
	def callback(self, data):
		# Information
		self.details = 0
		if data.severity >= 6:
			self.prefix = 2
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
		
		# Warning
		elif data.severity >= 4:
			self.prefix = 1
			# print(data.text)
		
		# Emergency
		else:
			self.prefix = 0
			if data.text.startswith("PreArm"):
				self.type = 5
				print("Cannot arm at the moment")
			else:
				print("This is emergency " + str(data.text))

    ############
    # Handling #
    ############	

	def throttletext(self,text):
		text.split()[1]
		if text.split()[1] == "armed":
			self.status = 1
		elif text.split()[1] == "disarmed":
			self.status = 0
		self.publishtext()

	def missiontext(self, text):
		self.status = int(text.split()[-1][1:])
		self.publishtext()
				
	# Plane 4.0
	# def missiontext4(self, text):
	# 	self.missionitem = text.split()[1]
	# 	self.missiondefine = text.split()[2]
	# 	print("Starting mission item " + str(self.missionitem) + ": " + str(self.missiondefine))
		
	def waypointtext(self, text):
		self.status = int(text.split()[2][1:])
		self.details = int(text.split()[4][:-1])
		self.publishtext()

	def transitiontext(self, text):
		if len(text) > 15:
			self.status = 1
			self.details = round(float(text.split()[3]),2)
		else:
			self.status = 0
		self.publishtext()

	def landtext(self,text):
		if text.split()[1] == "descend":
			self.status = 2
		elif text.split()[1] == "final":
			self.status = 1
		else:
			self.status = 0
		self.publishtext()

	def publishtext(self):
		yonahtext = YonahStatusText()
		yonahtext.prefix = self.prefix
		yonahtext.type = self.type
		yonahtext.status = self.status
		yonahtext.details = self.details
		self.pub.publish(yonahtext)

	def main(self):
		rospy.Subscriber("/mavros/statustext/recv/", StatusText, self.callback)
		rospy.spin()


if __name__ == '__main__':
	
	try:
		statustexthandler = StatusTextHandler()
		statustexthandler.main()

	except rospy.ROSInterruptException:
		pass
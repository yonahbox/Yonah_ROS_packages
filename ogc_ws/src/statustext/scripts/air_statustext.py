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
			elif data.text.startswith("Baro") or data.text.endswith("barometer"):
				self.type = 5
				self.barotext(data.text)
			elif data.text.startswith("GPS"):
				self.type = 6
				self.GPStext(data.text)
		
		# Warning
		elif data.severity >= 4:
			self.prefix = 1
			if data.text.startswith("VTOL") or data.text.endswith("mode"):
				self.type = 7
				self.VTOLtext(data.text)
		
		# Emergency
		else:
			self.prefix = 0
			if data.text.startswith("PreArm"):
				self.type = 8
				self.status = 0
			elif data.text.startswith("Mag"):
				self.type = 9
				self.status = 0
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
		if text.split()[-1] == "wait":
			self.status = 2
		elif len(text) > 15:
			self.status = 1
			self.details = round(float(text.split()[3]),2)
		else:
			self.status = 0
		self.publishtext()

	def landtext(self, text):
		if text.split()[1] == "descend":
			self.status = 2
		elif text.split()[1] == "final":
			self.status = 1
		else:
			self.status = 0
		self.publishtext()

	def barotext(self, text):
		if text.split()[0] == "Calibrating":
			self.status = 2
		elif text.split()[1] == "skipping":
			self.status = 1
		elif text.split()[-1] == "complete":
			self.status = 0
		self.publishtext()

	def GPStext(self, text):
		if text.split()[3] == "not":
			self.status = 0
			self.details = text.split()[1][:-1]
		self.publishtext()

	def VTOLtext(self, text):
		if text == "VTOL not available":
			self.status = 3
		elif text == "VTOL transition only in AUTO":
			self.status = 2
		elif text.startswith("Entered"):
			self.status = 1
		elif text.startswith("Exited"):
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
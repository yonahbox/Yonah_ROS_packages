#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText
from statustext.msg import YonahStatusText

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

class StatusTextGround:

	def __init__(self):
		rospy.init_node('gnd_statustext', anonymous=False)
		self.pub = rospy.Publisher('ogc/statustext_decoded', String, queue_size=5)
		self.message = ""

	def callback(self, data):
		# Throttle
		if data.type == 0:
			if data.status == 1:
				print("Armed")
			elif data.status == 0:
				print("Disarmed")
			else:
				print("Unknown command")
		# Mission
		elif data.type == 1:
			print("Executing " + str(navcommandtable.get(int(data.status), "Unknown command")))
		# Reached
		elif data.type == 2:
			print("Reached waypoint " + str(data.status) + " dist: " + str(int(data.details)))
		# Transition
		elif data.type == 3:
			if data.status == 1:
				print("Transition speed " + str(round(data.details,2)) + " reached")
			elif data.status == 0:
				print("Transition done")
			else:
				print("Unknown command")
		# Land
		elif data.type == 4:
			if data.status == 2:
				print("Land descend started")
			elif data.status == 1:
				print("Land final started")
			elif data.status == 0:
				print("Land complete")
			else:
				print("Unknown command")


	def main(self):
		rospy.Subscriber("ogc/statustext", YonahStatusText, self.callback)
		rospy.spin()

if __name__ == '__main__':
	
	try:
		statustextground = StatusTextGround()
		statustextground.main()

	except rospy.ROSInterruptException:
		pass
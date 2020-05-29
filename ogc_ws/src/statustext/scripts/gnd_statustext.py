#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText
from statustext.msg import YonahStatusText

texttable = {
	# Throttle
	0 : {
		0 : "Throttle disarmed", 
		1 : "Throttle armed",
	}, 
	# Mission
	1 : {
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
	}, 
	# Waypoint
	2 : {
		# No handling here
	}, 
	# Transition
	3 : {
		0 : "Transition done", 
		1 : "Reached transition speed ", 
		2 : "Transition speed not reached. Hovering", 
		}, 
	# Land
	4 : {
		0 : "Land complete", 
		1 : "Land final started", 
		2 : "Land descend started"
	}, 
	# Baro
	5 : {
		0 : "Barometer calibration complete", 
		1 : "Skipping baro calibration", 
		2 : "Calibrating barometer", 
	}, 
	# GPS
	6 : {
		0 : "GPS not found", 
	}, 
	# VTOL
	7 : {
		0 : "Exited VTOL mode", 
		1 : "Entered VTOL mode", 
		2 : "VTOL transition only in AUTO", 
		3 : "VTOL not available", 
	}, 
	# PreArm
	8 : {
		0 : "Cannot arm at the moment", 
	}, 
	# Compass
	9 : {
		0 : "Compass bad orientation", 
	}, 
}


class StatusTextGround:

	def __init__(self):
		rospy.init_node('gnd_statustext', anonymous=False)
		self.pub = rospy.Publisher('ogc/statustext_decoded', String, queue_size=5)
		self.message = ""

	def callback(self, data):
		try: 
			if data.type == 1:
				print("Executing " + texttable[data.type][data.status])
			elif data.type == 2:
				print("Reached waypoint " + str(data.status) + " dist " + str(data.details))
			elif data.type == 3 and data.status == 1:
				print(texttable[data.type][data.status] + str(data.detils))
			else:
				print(texttable[data.type][data.status])
		except KeyError:
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
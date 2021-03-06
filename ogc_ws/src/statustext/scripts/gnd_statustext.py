#!/usr/bin/env python3

'''
gnd_statustext: Receive and decode incoming statustext messages from aircraft
'''

# Copyright (C) 2020, Wang Huachen and Yonah (yonahbox@gmail.com)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import StatusText

prefixtable = {
	"i" : "[INFO]", 
	"w" : "[WARNING]", 
	"e" : "[ERROR]", 
}

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
		self.pub = rospy.Publisher('ogc/yonahtext', String, queue_size=5)
		self.message = ""

	def callback(self, data):
		try:
			headers = data.data.split()[:4]
			payload = data.data.split()[4] # Strip out message headers
			timestamp = str(data.data.split()[-1])
			pts, d = payload.split(".", 1)
			self.prefix = pts[0]
			self.type = int(pts[1])
			self.status = int(pts[2:])
			self.details = d
		except (ValueError, IndexError):
			self.pub.publish(",".join(headers) + "," + "Unknown statustext" + "," + timestamp)
			return

		try: 
			if self.type == 1:
				text = str(prefixtable[self.prefix] + " Executing " + texttable[self.type][self.status])
			elif self.type == 2:
				text = str(prefixtable[self.prefix] + " Reached waypoint " + str(self.status) + " dist " + str("%.0f" % float(self.details)))
			elif self.type == 3 and self.status == 1:
				text = str(prefixtable[self.prefix] + " " + texttable[self.type][self.status] + str(self.details))
			else:
				text = str(prefixtable[self.prefix] + " " + texttable[self.type][self.status])
		except KeyError:
			text = str("Unknown statustext")
		finally:
			self.pub.publish(",".join(headers) + "," + text + "," + timestamp)

	def main(self):
		rospy.Subscriber("ogc/from_despatcher/statustext", String, self.callback)
		rospy.spin()

if __name__ == '__main__':
	
	try:
		statustextground = StatusTextGround()
		statustextground.main()

	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python3

'''
rff: Performs return-from-flight routine via safety button, LED and buzzer
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

import time
import rospy
import RuTOS
import threading
import Adafruit_BBIO.GPIO as GPIO
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import State
from std_msgs.msg import String
from despatcher.msg import LinkMessage
from waypoint import WP

missionlist = []

class RFF:

	def __init__(self):
		# Set defaults
		self.armStatus = True
		self.hopStatus = False
		self.Ack = False

		# Check arm status of aircraft
		rospy.Subscriber('/mavros/state', State, self.checkDisarm)
		# Check hop status of aircraft
		rospy.Subscriber('/ogc/to_rff', String, self.checkHop)
		# Wait for MAVROS services
		rospy.wait_for_service('mavros/set_mode')
		rospy.wait_for_service('mavros/cmd/arming')

		self.pub_to_aircraft = rospy.Publisher('ogc/from_telegram', String, queue_size = 5)

	def main(self):
		if self.armStatus == True or self.hopStatus == False:
			rospy.logwarn("Button disabled")
			return
		rospy.loginfo("Loading next set of waypoints.")
		self.pub_to_aircraft.publish("i 0 10 mission next " + str(rospy.get_rostime().secs))
		time.sleep(5)
		if self.hopStatus == False or self.Ack == False:
			rospy.loginfo("Unable to load")
			return
		mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
		while not mode(custom_mode = "AUTO").mode_sent:
			rospy.logerr("Failed to set mode to AUTO. Trying again.")
			time.sleep(3)
		rospy.loginfo("MODE = AUTO")
		rospy.loginfo("Arming in 10 seconds. Please stand clear.")
		time.sleep(10)
		arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
		while not arm(1).success:
			rospy.logwarn("Failed to arm throttle. Trying again. Please continue to stay clear of aircraft.")
			time.sleep(3)
		rospy.loginfo("THROTTLE ARMED")
		self.armStatus = True
		self.Ack = False

	def checkDisarm(self, data):
		self.armStatus = data.armed

	def checkHop(self, data):
		text = data.data.split()
		if text[0] == "hop":
			self.hopStatus = eval(text[1])
		elif text[0] == "ack":
			self.Ack = True


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
			buzzer_pin = "P8_11"
			GPIO.setup(buzzer_pin, GPIO.OUT)
			for i in range(300):
				GPIO.output(buzzer_pin, GPIO.HIGH)
				time.sleep(0.0003)
				GPIO.output(buzzer_pin, GPIO.LOW)
				time.sleep(0.0003)
			time.sleep(0.5)
			if self.stopwarn:
				GPIO.cleanup()
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
	rff = RFF()

	try:
		while True:
			# Do nothing while aircraft is not in hop mode
			while not rff.hopStatus:
				time.sleep(5)
			rospy.loginfo("Plane is now in hop mode")
			# Do nothing if aircraft is armed
			while rff.armStatus:
				time.sleep(5)
			rospy.loginfo("Plane has been disarmed.")
			button = Button()
			button.press()

	except KeyboardInterrupt:
		rospy.signal_shutdown("Shutting down")

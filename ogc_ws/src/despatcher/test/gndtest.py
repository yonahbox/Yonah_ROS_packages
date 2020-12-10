#!/usr/bin/env python3

'''
gnd_test: Test node to ensure that gnd_despatcher node works properly

Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Standard Library
from datetime import datetime

# ROS/Third-Party
import rospy
from despatcher.msg import RegularPayload
from std_msgs.msg import String
from despatcher.msg import LinkMessage

pub_to_despatcher = rospy.Publisher('ogc/to_despatcher', LinkMessage, queue_size=5)

def display_regular_payload(data):
    '''Receive input from ogc/from_despatcher topic and print it to terminal'''
    if data.is_aircraft:
        rospy.loginfo("Msg from Aircraft " + str(data.vehicle_no))
    else:
        rospy.loginfo("Msg from GCS " + str(data.vehicle_no))
    rospy.loginfo("Airspeed: " + str(data.airspeed))
    rospy.loginfo("Altitude: " + str(data.alt))
    rospy.loginfo("Armed: " + str(data.armed))
    rospy.loginfo("Mode: " + str(data.mode))
    rospy.loginfo("Groundspeed: " + str(data.groundspeed))
    rospy.loginfo("Fuel: " + str(data.fuel))
    rospy.loginfo("Battery: " + str(data.batt))
    rospy.loginfo("GPS Coords: " + str(data.lat) + ", " + str(data.lon))
    rospy.loginfo("Throttle: " + str(data.throttle))
    rospy.loginfo("VTOL Status: " + str(data.vtol))
    rospy.loginfo("WP Reached: " + str(data.wp))
    rospy.loginfo("Vibration: " + str(data.vibe))
    rospy.loginfo("Transmit Time: " + str(datetime.fromtimestamp(data.header.stamp.secs)))

def display_ondemand_payload(data):
    '''Receive input from ogc/from_despatcher topic and print it to terminal'''
    rospy.loginfo(str(data.data))

def display_statustext(data):
    rospy.loginfo("Statustext: " + str(data.data))

def send_msgs(data):
    '''Send messages from user input (in terminal) to screen'''
    cmd = LinkMessage()
    cmd.uuid = 0
    cmd.id = 1 # Temp ID
    cmd.data = input()
    pub_to_despatcher.publish(cmd)

def client():
    rospy.init_node('gnd_test', anonymous=False)
    rospy.Subscriber('ogc/from_despatcher/regular', RegularPayload, display_regular_payload)
    rospy.Subscriber('ogc/from_despatcher/ondemand', String, display_ondemand_payload)
    rospy.Subscriber('ogc/from_despatcher/statustext', String, display_statustext)
    message_sender = rospy.Timer(rospy.Duration(0.5), send_msgs)
    rospy.spin()
    message_sender.shutdown()

if __name__=='__main__':
    client()
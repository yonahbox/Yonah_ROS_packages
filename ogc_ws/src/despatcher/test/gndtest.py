#!/usr/bin/env python3

'''
gnd_test.py

Test script to ensure that gnd_despatcher node works properly

Lau Yan Han and Yonah, Apr 2020

'''

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from despatcher.msg import regular_payload
from std_msgs.msg import String

def display_regular_payload(data):
    '''Receive input from ogc/from_despatcher topic and print it to terminal'''
    rospy.loginfo("Airspeed: " + str(data.airspeed))
    rospy.loginfo("Altitude: " + str(data.alt))
    rospy.loginfo("Armed: " + str(data.armed))
    rospy.loginfo("Groundspeed: " + str(data.groundspeed))
    rospy.loginfo("GPS Coords: " + str(data.lat) + ", " + str(data.lon))
    rospy.loginfo("Throttle: " + str(data.throttle))
    rospy.loginfo("VTOL Status: " + str(data.vtol))
    rospy.loginfo("WP Reached: " + str(data.wp))

def display_ondemand_payload(data):
    '''Receive input from ogc/from_despatcher topic and print it to terminal'''
    rospy.loginfo("On-demand message: " + str(data.data))

def client():
    rospy.init_node('gnd_test', anonymous=False)
    rospy.Subscriber('ogc/from_despatcher/regular', regular_payload, display_regular_payload)
    rospy.Subscriber('ogc/from_despatcher/ondemand', String, display_ondemand_payload)
    rospy.spin()

if __name__=='__main__':
    client()
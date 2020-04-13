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
from despatcher.msg import on_demand_payload
from std_msgs.msg import String

def display_regular_payload(data):
    '''Receive input from ogc/from_despatcher topic and print it to terminal'''
    print(data)

def client():
    rospy.init_node('gnd_test', anonymous=False)
    rospy.Subscriber('ogc/from_despatcher', regular_payload, display_regular_payload)
    rospy.spin()

if __name__=='__main__':
    client()
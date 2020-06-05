#!/usr/bin/env python3

'''
gnd_despatcher: Bridge between RQT Ground Console and the Ops Ground Control Links (Telegram, SMS, SBD)

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
import subprocess

# ROS/Third-Party
import rospy
from despatcher.msg import RegularPayload
from std_msgs.msg import String

# Local
import regular

class gnddespatcher():

    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('gnd_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', String, queue_size = 5) # Link to SBD node
        self.pub_to_rqt_regular = rospy.Publisher('ogc/from_despatcher/regular', RegularPayload, queue_size=5)
        self.pub_to_rqt_ondemand = rospy.Publisher('ogc/from_despatcher/ondemand', String, queue_size=5)
        self.recv_msg = "" # Stores outgoing Ground-to-Air message

        # Temp params for msg headers
        # To-do: Work on air/gnd identifiers whitelist file
        self.is_air = 1 # 1 if aircraft, 0 if GCS
        self.id = 1 # ID number
        self.severity = "i" # Msg severity level
        self.prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous recv msg

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################
    
    def handle_outgoing_msgs(self, data):
        '''Check that outgoing G2A messages are valid before forwarding them to the links'''
        whitelisted_prefixes = ["ping", "sms", "arm", "disarm", "mode", "wp"]
        if data.data.split()[0] not in whitelisted_prefixes:
            self.pub_to_rqt_ondemand.publish("Invalid command: " + data.data)
        else:
            # Add msg headers
            # To-do: Add decode msg on air despatcher side to strip out prefixes b4 interpreting msg
            #msg = self.severity + " " + str(self.is_air) + " " + str(self.id) + \
            #    " " + data.data + " " + str(rospy.get_rostime().secs)
            # To-do: Add if-else statement to handle 3 links, add feedback for sms node on successful publish of msg
            self.pub_to_sms.publish(data.data)
            self.pub_to_sbd.publish(data.data)
    
    ###########################################
    # Handle Air-to-Ground (A2G) messages
    ###########################################

    def is_new_msg(self, timestamp):
        '''Return true is incoming msg is a new msg'''
        if timestamp < self.prev_transmit_time:
            return False
        else:
            self.prev_transmit_time = timestamp
            return True
    
    def check_incoming_msgs(self, data):
        '''Check for incoming A2G messages from ogc/from_sms, from_sbd or from_telegram topics'''
        entries = data.data.split()
        try:
            if not self.is_new_msg(int(entries[-1])):
                return
        except ValueError:
            rospy.logerr("Msg doesn't have UNIX timestamp!")
            return
        if regular.is_regular(entries):
            # Check if it is regular payload
            msg = regular.convert_to_rosmsg(entries)
            self.pub_to_rqt_regular.publish(msg)
        else:
            self.pub_to_rqt_ondemand.publish(data.data)
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/to_despatcher", String, self.handle_outgoing_msgs)
        rospy.spin()

if __name__=='__main__':
    run = gnddespatcher()
    run.client()
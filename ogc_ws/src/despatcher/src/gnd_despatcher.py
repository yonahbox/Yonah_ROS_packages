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

REGULAR_PAYLOAD_LEN = 9

class gnddespatcher():

    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('gnd_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', String, queue_size = 5) # Link to SBD node
        self.pub_to_rqt_regular = rospy.Publisher('ogc/from_despatcher/regular', RegularPayload, queue_size=5)
        self.pub_to_rqt_ondemand = rospy.Publisher('ogc/from_despatcher/ondemand', String, queue_size=5)
        self.msg = "" # Stores incoming Air-to-Ground message
        self.recv_msg = "" # Stores outgoing Ground-to-Air message

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################

    def handle_outgoing_msgs(self, data):
        '''Check that outgoing G2A messages are valid before forwarding them to the links'''
        whitelisted_prefixes = ["ping", "sms", "statustext", "arm", "disarm", "mode", "wp"]
        if data.data.split()[0] not in whitelisted_prefixes:
            self.pub_to_rqt_ondemand.publish("Invalid command: " + data.data)
        else:
            # To-do: Add if-else statement to handle 3 links, add feedback for sms node on successful publish of msg
            self.pub_to_sms.publish(data.data)
            self.pub_to_sbd.publish(data.data)
    
    ###########################################
    # Handle Air-to-Ground (A2G) messages
    ###########################################

    def check_incoming_msgs(self, data):
        '''Check for incoming A2G messages from ogc/from_sms, from_sbd or from_telegram topics'''
        self.msg = data.data
        if not self.handle_regular_payload():
            self.handle_ondemand_payload()

    def handle_regular_payload(self):
        '''Check if incoming message is a regular payload, and handle it accordingly'''
        global REGULAR_PAYLOAD_LEN
        entries = self.msg.split()
        # Check if incoming message is of the regular payload format
        if not len(entries) == REGULAR_PAYLOAD_LEN:
            return False
        # Decode regular payload
        msg = RegularPayload()
        msg.airspeed = float(entries[0])
        msg.alt = float(entries[1])
        msg.armed = bool(entries[2])
        msg.groundspeed = float(entries[3])
        msg.lat = float(entries[4])
        msg.lon = float(entries[5])
        msg.throttle = float(entries[6])
        msg.vtol = bool(entries[7])
        msg.wp = int(entries[8])
        self.pub_to_rqt_regular.publish(msg)
        return True

    def handle_ondemand_payload(self):
        '''Publish on-demand and miscellaneous messages to be displayed on rqt's console'''
        # To-do: Add some checking mechanism. Right now, all incoming msgs are strings with no fixed format
        self.pub_to_rqt_ondemand.publish(self.msg)
    
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
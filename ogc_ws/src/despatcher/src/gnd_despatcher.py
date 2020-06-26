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

# ROS/Third-Party
import rospy

from std_msgs.msg import String
from despatcher.msg import RegularPayload
from despatcher.msg import LinkMessage

# Local
import regular

class gnddespatcher():

    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('gnd_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', LinkMessage, queue_size = 5) # Link to SMS node
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', LinkMessage, queue_size = 5) # Link to SBD node
        self.pub_to_telegram = rospy.Publisher('ogc/to_telegram', LinkMessage, queue_size = 5) # Link to Telegram node
        self.pub_to_rqt_regular = rospy.Publisher('ogc/from_despatcher/regular', RegularPayload, queue_size=5)
        self.pub_to_rqt_ondemand = rospy.Publisher('ogc/from_despatcher/ondemand', String, queue_size=5)
        self.pub_to_statustext = rospy.Publisher('ogc/from_despatcher/statustext', String, queue_size=5)

        # Link switching
        self.link_select = 2 # 0 = Tele, 1 = SMS, 2 = SBD

        # Gnd Identifiers and msg headers (attached to outgoing msgs)
        self._is_air = 0 # 1 = Aircraft, 0 = GCS. Obviously, gnd despatcher should be on a GCS...
        self._id = rospy.get_param("~self_id") # Our GCS ID
        self._severity = "i" # Outgoing msg severity level

        # Used to handle incoming msgs
        self._prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous recv msg (incoming msg)

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################
    
    def handle_outgoing_msgs(self, data):
        '''Check that outgoing G2A messages are valid before forwarding them to the links'''
        whitelisted_prefixes = ["ping", "sms", "statustext", "arm", "disarm", "mode", "wp"]
        if data.data.split()[0] not in whitelisted_prefixes:
            self.pub_to_rqt_ondemand.publish("Invalid command: " + data.data)
        else:
            msg = LinkMessage()
            msg.id = data.id
            # Add msg headers
            msg.data = self._severity + " " + str(self._is_air) + " " + str(self._id) + \
                " " + data.data + " " + str(rospy.get_rostime().secs)
            if self.link_select == 0:
                self.pub_to_telegram.publish(msg)
            elif self.link_select == 1:
                self.pub_to_sms.publish(msg)
            else:
                self.pub_to_sbd.publish(msg)
            self.pub_to_rqt_ondemand.publish("Command sent: " + data.data)

    ###########################################
    # Handle Air-to-Ground (A2G) messages
    ###########################################

    def _is_new_msg(self, timestamp):
        '''Return true is incoming msg is a new msg'''
        if timestamp < self._prev_transmit_time:
            return False
        else:
            self._prev_transmit_time = timestamp
            return True
    
    def check_incoming_msgs(self, data):
        '''Check for incoming A2G messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            # Handle msg prefixes
            entries = data.data.split()
            sender_timestamp = int(entries[-1])
            sender_msgtype = str(entries[0])
            if not self._is_new_msg(sender_timestamp):
                # Check if it is new msg
                return
            if regular.is_regular(sender_msgtype, len(entries)):
                # Check if it is regular payload
                msg = regular.convert_to_rosmsg(entries)
                self.pub_to_rqt_regular.publish(msg)
            else:
                if sender_msgtype == 's':
                    # Check if it is statustext
                    self.pub_to_statustext.publish(data.data)
                else:
                    self.pub_to_rqt_ondemand.publish(data.data)
        except (ValueError, IndexError):
            rospy.logerr("Invalid message format!")
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.handle_outgoing_msgs)
        rospy.spin()

if __name__=='__main__':
    run = gnddespatcher()
    run.client()

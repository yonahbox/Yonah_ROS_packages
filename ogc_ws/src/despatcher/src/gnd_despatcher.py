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

from std_msgs.msg import UInt8, String
from despatcher.msg import RegularPayload
from despatcher.msg import LinkMessage

# Local
import regular
from g2a import recognised_commands
from headers import headerhandler

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
        self.file_to_telegram = rospy.Publisher('ogc/to_telegram/file', LinkMessage, queue_size = 5) # Link to Telegram node

        # Link switching
        self.link_select = rospy.get_param("~link_select") # 0 = Tele, 1 = SMS, 2 = SBD

        # Gnd Identifiers and msg headers (attached to outgoing msgs)
        self._is_air = 0 # 1 = Aircraft, 0 = GCS. Obviously, gnd despatcher should be on a GCS...
        self._id = rospy.get_param("~self_id") # Our GCS ID
        self._valid_ids = rospy.get_param("~valid_ids")

        # Msg header handling
        self._header = headerhandler()

        # Intervals btwn heartbeat msgs
        self._interval_1 = rospy.get_param("~interval_1")
        self._interval_2 = rospy.get_param("~interval_2")
        self._interval_3 = rospy.get_param("~interval_3")
        self._tele_interval = self._interval_1
        self._sms_interval = self._interval_2

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################
    
    def handle_outgoing_msgs(self, data):
        '''Check that outgoing G2A messages are valid before forwarding them to the links'''
        if data.data.split()[0] not in recognised_commands:
            self.pub_to_rqt_ondemand.publish("Invalid command: " + data.data)
        else:
            msg = LinkMessage()
            msg.id = data.id
            # Add msg headers
            prefixes = ["i", self._is_air, self._id]
            msg.data = self._header.attach_headers(prefixes, [rospy.get_rostime().secs], "HB")
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
    
    def check_incoming_msgs(self, data):
        '''Check for incoming A2G messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            # Handle msg prefixes
            msgtype, devicetype, sysid, timestamp, entries \
                = self._header.split_headers(data.data)
            if not self._header.is_new_msg(timestamp):
                # Check if it is new msg
                return
            if regular.is_regular(msgtype, len(entries)):
                # Check if it is regular payload
                msg = regular.convert_to_rosmsg(entries)
                self.pub_to_rqt_regular.publish(msg)
            else:
                if msgtype == 's':
                    # Check if it is statustext
                    self.pub_to_statustext.publish(data.data)
                elif msgtype == 'm':
                    # Check if it is mission update message
                    reqFile = LinkMessage()
                    reqFile.id = sysid
                    if entries == ["No", "update", "required"]:
                        return
                    for i in entries:
                        reqFile.data = i
                        self.file_to_telegram.publish(reqFile)
                        rospy.sleep(1)
                else:
                    self.pub_to_rqt_ondemand.publish(data.data)
        except (ValueError, IndexError, TypeError):
            rospy.logerr("Invalid message format!")
    
    #########################################
    # Handle Misc (e.g. switcher) messages
    #########################################

    def _prep_heartbeat(self):
        '''Prepare a heartbeat msg'''
        prefixes = ["h", self._is_air, self._id]
        return self._header.attach_headers(prefixes, [rospy.get_rostime().secs], "HB")
    
    def send_heartbeat_tele(self, data):
        msg = LinkMessage()
        msg.data = self._prep_heartbeat()
        for ids in self._valid_ids:
            msg.id = ids
            self.pub_to_telegram.publish(msg)
        rospy.sleep(self._tele_interval)

    def send_heartbeat_sms(self, data):
        msg = LinkMessage()
        msg.data = self._prep_heartbeat()
        for ids in self._valid_ids:
            msg.id = ids
            self.pub_to_sms.publish(msg)
        rospy.sleep(self._sms_interval)

    def check_switcher(self, data):
        '''Obtain updates from switcher node'''
        self.link_select = data.data
        if self.link_select == 2:
            self._tele_interval = self._interval_3
            self._sms_interval = self._interval_3
        elif self.link_select == 1:
            self._tele_interval = self._interval_2
            self._sms_interval = self._interval_2
            self.sms_sender = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat_sms)
        elif self.link_select == 0:
            self._tele_interval = self._interval_1
            try:
                self.sms_sender.shutdown()
            except:
                pass

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.handle_outgoing_msgs)
        rospy.Subscriber("ogc/from_switcher", UInt8, self.check_switcher)
        self.tele_sender = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat_tele)
        rospy.spin()
        self.tele_sender.shutdown()

if __name__=='__main__':
    run = gnddespatcher()
    run.client()

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
import os
import rospkg

from std_msgs.msg import String, UInt8MultiArray
from despatcher.msg import RegularPayload
from despatcher.msg import LinkMessage
from identifiers.srv import GetSelfDetails, GetIds

# Local
import regular
import headers
import feedback_util
from g2a import recognised_commands

TELE = 0
SMS = 1
SBD = 2

class aircraft():
    '''Monitor link status and heartbeat for each aircraft'''
    
    def __init__(self, air_id):
        self._air_id = air_id
        self._link = TELE
        self._tele_interval = rospy.get_param("~interval_1")
        self._sms_interval = rospy.get_param("~interval_2")
        self._pub_to_telegram = rospy.Publisher('ogc/to_telegram', LinkMessage, queue_size = 5)
        self._pub_to_sms = rospy.Publisher('ogc/to_sms', LinkMessage, queue_size = 5)
        self.tele_sender = rospy.Timer(rospy.Duration(0.5), self._send_heartbeat_tele)

        rospy.wait_for_service("identifiers/get/valid_ids")
        rospy.wait_for_service("identifiers/self/self_id")

        ids_get_self_id = rospy.ServiceProxy("identifiers/self/self_id", GetSelfDetails)
        self._id = ids_get_self_id().data_int


    def _prep_heartbeat(self):
        '''Prepare a heartbeat msg'''
        prefixes = ["h", 0, self._id, 0]
        return headers.attach_headers(prefixes, [rospy.get_rostime().secs], "HB")
    
    def _send_heartbeat_tele(self, data):
        msg = LinkMessage()
        msg.uuid = 0
        msg.data = self._prep_heartbeat()
        msg.id = self._air_id
        self._pub_to_telegram.publish(msg)
        rospy.sleep(self._tele_interval)

    def _send_heartbeat_sms(self, data):
        msg = LinkMessage()
        msg.uuid = 0
        msg.data = self._prep_heartbeat()
        msg.id = self._air_id
        self._pub_to_sms.publish(msg)
        rospy.sleep(self._sms_interval)

    def switch_link(self, link):
        self._link = link
        if self._link == SBD:
            self._tele_interval = rospy.get_param("~interval_2")
            self._sms_interval = rospy.get_param("~interval_3")
        elif self._link == SMS:
            self._tele_interval = rospy.get_param("~interval_2")
            self._sms_interval = rospy.get_param("~interval_2")
            self.sms_sender = rospy.Timer(rospy.Duration(0.5), self._send_heartbeat_sms)
        elif self._link == TELE:
            self._tele_interval = rospy.get_param("~interval_1")
            if hasattr(self, 'sms_sender'):
                self.sms_sender.shutdown()
    
    def link_status(self):
        return self._link

    def kill_timers(self):
        rospy.loginfo("Despatcher: Restarting aircraft timers")
        self.tele_sender.shutdown()
        if hasattr(self, 'sms_sender'):
            self.sms_sender.shutdown()

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
        self.pub_to_timeout = rospy.Publisher('ogc/to_timeout', LinkMessage, queue_size = 5) # Link to Timeout Module

        rospy.wait_for_service("identifiers/self/self_id")
        rospy.wait_for_service("identifiers/get/valid_ids")
        ids_get_self_id = rospy.ServiceProxy("identifiers/self/self_id", GetSelfDetails)
        ids_get_valid_ids = rospy.ServiceProxy("identifiers/get/valid_ids", GetIds)

        rospy.Subscriber("ogc/identifiers/valid_ids", UInt8MultiArray, self.update_valid_ids_cb)

        # Gnd Identifiers and msg headers (attached to outgoing msgs)
        self._is_air = 0 # 1 = Aircraft, 0 = GCS. Obviously, gnd despatcher should be on a GCS...
        self._id = ids_get_self_id().data_int # Our GCS ID
        self._severity = "i" # Outgoing msg severity level

        # Msg headers and valid aircrafts to send to
        self._valid_ids = ids_get_valid_ids().ids
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)
        self._aircrafts = dict()
        for i in self._valid_ids:
            self._aircrafts[i] = aircraft(i)

    def update_valid_ids_cb(self, msg):
        '''Callback function to obtain updated list of valid ids from the admin server'''
        self._valid_ids = [i for i in msg.data]
        # clear and recreate self._new_msg_chk and self._aircrafts, since they are dependent on valid id list
        del self._new_msg_chk
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)
        for j in self._aircrafts.keys():
            self._aircrafts[j].kill_timers()
        del self._aircrafts
        self._aircrafts = {}
        for i in self._valid_ids:
            self._aircrafts[i] = aircraft(i)

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################

    def handle_outgoing_msgs(self, data):
        '''Check that outgoing G2A messages are valid before forwarding them to the links'''
        dummy_prefixes = ["i", 1, data.id, 0] # Dummy prefixes for publishing local msgs to on_demand topic
        feedback_to_rqt = ""
        if data.data.split()[0] not in recognised_commands:
            feedback_to_rqt = "Invalid command: " + data.data
        else:
            msg = LinkMessage()
            msg.uuid = data.uuid
            msg.id = data.id
            # Add msg headers
            prefixes = ["i", self._is_air, self._id, data.uuid]
            msg.data = headers.attach_headers(prefixes, [rospy.get_rostime().secs], data.data)
            try:
                link = self._aircrafts[data.id].link_status()
                if link == TELE:
                    self.pub_to_telegram.publish(msg)
                    feedback_to_rqt = "Command sent to Tele: " + data.data
                elif link == SMS:
                    self.pub_to_sms.publish(msg)
                    feedback_to_rqt = "Command sent to SMS: " + data.data
                elif link == SBD:
                    self.pub_to_sbd.publish(msg)
                    feedback_to_rqt = "Command sent to SBD: " + data.data
                else:
                    rospy.logerr("Despatcher: Invalid link")
                    feedback_to_rqt = "Invalid Link. Command " + data.data + " not sent"
            except KeyError:
                rospy.logerr("Despatcher: Invalid aircraft ID")
                feedback_to_rqt = "Invalid Aircraft ID. Command " + data.data + " not sent"
            self.pub_to_rqt_ondemand.publish(\
                headers.attach_headers(dummy_prefixes, [rospy.get_rostime().secs], feedback_to_rqt))

    ###########################################
    # Handle Air-to-Ground (A2G) messages
    ###########################################
    
    def check_incoming_msgs(self, data):
        '''Check for incoming A2G messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            # Handle msg prefixes
            msgtype, devicetype, sysid, uuid, timestamp, entries \
                = headers.split_headers(data.data)
            if not self._new_msg_chk.is_new_msg(timestamp, sysid):
                # Check if it is new msg
                return
            # Check if it is regular msg. This requires a list of the msg with all its headers attached
            data_list = data.data.split() # Msg + Headers, split into a list
            if regular.is_regular(msgtype, len(data_list)):
                # Check if it is regular payload
                msg = regular.convert_to_rosmsg(data_list)
                self.rqt_recovery_log(data_list, sysid)
                self.pub_to_rqt_regular.publish(msg)
            else:
                # Check if the incoming message is an acknowledgment message
                if uuid != 0:
                    ack_msg = LinkMessage()
                    ack_msg.uuid = uuid
                    ack_msg.data = ""
                    ack_msg.id = 1
                    ack = feedback_util.ack_converter(ack_msg, 2)
                    if ack != None:
                        self.pub_to_timeout.publish(ack)
                    return 0 # Prevent the message to get sent through ondemand
                if msgtype == 's':
                    # Check if it is statustext
                    self.pub_to_statustext.publish(data.data)
                elif msgtype == 'm':
                    # Check if it is mission update message
                    reqFile = LinkMessage()
                    reqFile.uuid = 0
                    reqFile.id = sysid
                    if entries == ["No", "update", "required"]:
                        return
                    for i in entries:
                        reqFile.data = "Waypoints/" + i
                        self.file_to_telegram.publish(reqFile)
                        rospy.sleep(1)
                else:
                    self.pub_to_rqt_ondemand.publish(data.data)
        except (ValueError, IndexError, TypeError):
            rospy.logerr("Despatcher: Invalid message format")
    
    ####################################################
    # Handle Misc (e.g. switcher, rqt recovery) messages
    ####################################################

    def check_switcher(self, data):
        '''Obtain updates from switcher node'''
        try:
            msglist = data.data.split()
            id = int(msglist[0]) # Msg format: aircraft id + link no
            link = int(msglist[1])
            self._aircrafts[id].switch_link(link)

            # Notify RQT of the link switch
            dummy_prefixes = ["i", 1, id, 0] # Dummy prefixes for publishing local msgs to on_demand topic
            self.pub_to_rqt_ondemand.publish(\
                headers.attach_headers(dummy_prefixes, [rospy.get_rostime().secs], "LinkSwitch " + str(link)))

        except:
            rospy.logerr("Despatcher: Invalid message")

    def rqt_recovery_log(self, reg_payload_list, aircraft_id):
        filename = "rqt_log.txt"
        path = os.path.join(rospkg.RosPack().get_path("yonah_rqt"), "src/yonah_rqt", filename)
        with open(path, 'r') as lines:
            data = lines.readlines()
        if len(data) < aircraft_id:
            log = open(path, 'a')
            for i in range (aircraft_id - len(data)):
                log.write("None\n")
            log.close()
        with open(path, 'r') as lines:
            data = lines.readlines()
        data[aircraft_id - 1] = " ".join(reg_payload_list) +"\n"
        with open(path, 'w') as files:
            files.writelines(data)
            files.close()

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.handle_outgoing_msgs)
        rospy.Subscriber("ogc/from_switcher", String, self.check_switcher)
        rospy.spin()

if __name__=='__main__':
    run = gnddespatcher()
    run.client()

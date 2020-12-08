#!/usr/bin/env python3

'''
air_despatcher: Bridge between MAVROS and the Ops Ground Control Links (Telegram, SMS, SBD)

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
import csv
from pathlib import Path
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import WaypointList
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import Vibration
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.srv import WaypointPush
from std_msgs.msg import String
from statustext.msg import YonahStatusText
from despatcher.msg import LinkMessage

from identifiers.srv import GetSelfDetails, GetIds

# Local
from regular import air_payload
import waypoint
import g2a


class airdespatcher():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('air_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', LinkMessage, queue_size = 5) # Link to SMS node
        self.pub_to_telegram = rospy.Publisher('ogc/to_telegram', LinkMessage, queue_size = 5) # Link to telegram node      
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', LinkMessage, queue_size = 5) # Link to SBD node
        self.pub_to_rff = rospy.Publisher('ogc/to_rff', String, queue_size = 5)

        # Msg handlers
        self._msg = "" # Stores outgoing msg Air-to-Ground message
        self._recv_msg = list() # Stores incoming Ground-to-Air message in list form
        self._regular_payload_flag = True # Whether we should send regular payload to Ground Control
        self._statustext_flag = True # Whether we should send status texts to Ground Control
        self.payloads = air_payload() # Handler for regular and on-demand payloads
        self.link_select = rospy.get_param("~link_select") # 0 = Tele, 1 = SMS, 2 = SBD
        self._prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous incoming msg


        rospy.wait_for_service("identifiers/self/self_id")
        rospy.wait_for_service("identifiers/get/valid_ids")
        ids_get_self_id = rospy.ServiceProxy("identifiers/self/self_id", GetSelfDetails)
        ids_get_valid_ids = rospy.ServiceProxy("identifiers/get/valid_ids", GetIds)

        # Air Identifiers (attached to outgoing msgs)
        self._is_air = 1 # 1 = Aircraft, 0 = GCS. Obviously, air despatcher should be on an aircraft...
        self._id = ids_get_self_id().data_int
        # self._id = rospy.get_param("~self_id") # Our aircraft ID

        # Ground Identifiers. For now we assume only one GCS
        # self.ground_id = rospy.get_param("~ground_ids")[0]
        self.ground_id = ids_get_valid_ids().ids[0]

        # Intervals btwn msgs
        self._interval_1 = rospy.get_param("~interval_1") # Short time interval (seconds) for regular payload
        self._interval_2 = rospy.get_param("~interval_2") # Long time interval (seconds) for regular payload
        self._tele_interval = self._interval_1 # Interval (seconds) for regular payload over telegrams
        self._sms_interval = self._interval_2 # Interval (seconds) for regular payload over sms
        # Note that there is no sbd interval. This interval is controlled by sbd link node
      
        # Mission params
        self.hop = False
        self.missionlist = []
        self.wpfolder = rospy.get_param('~waypoint_folder', '/home/ubuntu/Waypoints/')

        # Wait for MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')
        rospy.wait_for_service('mavros/mission/push')

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################

    def _is_new_msg(self, timestamp):
        '''Return true is incoming msg is a new msg'''
        if timestamp < self._prev_transmit_time:
            return False
        else:
            self._prev_transmit_time = timestamp
            return True
    
    def check_incoming_msgs(self, data):
        '''Check for incoming G2A messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            rospy.loginfo("Received \"" + data.data + "\"")
            # Handle msg headers
            self._recv_msg = data.data.split()
            sender_timestamp = int(self._recv_msg[-1])
            if not self._is_new_msg(sender_timestamp):
                return
            self._recv_msg = self._recv_msg[3:-1] # Strip out msg headers
            # Go through series of checks
            if "ping" in self._recv_msg:
                # self._check_ping()
                g2a.check_ping(self)
            elif "sms" in self._recv_msg:
                g2a.check_sms(self)
            elif "statustext" in self._recv_msg:
                g2a.check_statustext(self)
            elif "arm" in self._recv_msg or "disarm" in self._recv_msg:
                g2a.check_arming(self)
            elif "mode" in self._recv_msg:
                g2a.check_mode(self)
            elif "wp" in self._recv_msg or "mission" in self._recv_msg:
                g2a.check_mission(self)
        except(rospy.ServiceException):
            rospy.logwarn("Service Call Failed")
        except (ValueError, IndexError):
            rospy.logerr("Invalid message format")


    #########################################
    # Handle Air-to-Ground (A2G) messages
    #########################################
    
    def _send_ack(self):
        '''Send a acknowledgement of the msg in self._recv_msg'''
        self._msg = ' '.join(self._recv_msg)
        self.sendmsg("a")
    
    def _attach_headers(self, severity):
        '''Attach message headers (prefixes and suffixes'''
        self._msg = severity + " " + str(self._is_air) + " " + str(self._id) + " " + \
            self._msg + " " + str(rospy.get_rostime().secs)

    def _send_regular_payload_sms(self):
        '''Send regular payload over sms link'''
        message = LinkMessage()
        message.data = self._msg
        message.id = self.ground_id
        self.pub_to_sms.publish(message)
    
    def _send_regular_payload_sbd(self):
        '''Send regular payload over SBD Satcomms link'''
        message = LinkMessage()
        message.data = self._msg
        message.id = self.ground_id
        self.pub_to_sbd.publish(message)

    def _send_regular_payload_tele(self):
        '''Send regular payload over Telegram link'''
        message = LinkMessage()
        message.data = self._msg
        message.id = self.ground_id
        self.pub_to_telegram.publish(message)
    
    def sendmsg(self, severity):
        '''Send any msg that's not a regular payload'''
        self._attach_headers(severity)
        message = LinkMessage()
        message.id = self.ground_id
        message.data = self._msg
        print(message.data)
        if self.link_select == 0:
            self.pub_to_telegram.publish(message)
        elif self.link_select == 1:
            self.pub_to_sms.publish(message)
        else:
            self.pub_to_sbd.publish(message)
        

    def check_alerts(self, data):
        '''Check for special alerts'''
        # Vibrations. Vibes above 30 are considered bad
        bad_vibes = 30
        previous_vibes = self.payloads.ping_entries["vibe"]
        i = 0
        while i < 3:
            if self.payloads.ping_entries["vibe"][i] >= bad_vibes and previous_vibes[i] < bad_vibes:
                self._msg = "Vibrations are high"
                self.sendmsg("w")
                break
            i = i + 1
        # Add more special alert checks here
        # @TODO: Better msg prefix system
    
    def get_status_text(self, data):
        '''Obtain status text messages from ogc/statustext'''
        statustextmsg = str(data.prefix) + str(data.type) + str(data.status) + "." + str(round(data.details, 1))
        self.payloads.ping_entries["msg"] = statustextmsg
        # Send new status texts to Ground Control
        if self._statustext_flag:
            self._msg = statustextmsg
            self.sendmsg("s")
    
    def send_regular_payload(self, data):
        if self._regular_payload_flag == False:
            return
        '''Send regular payload over one of the three links: SMS, SBD or Telegram'''
        self._msg = self.payloads.truncate_regular_payload()
        self._attach_headers("r")
        if self.link_select == 0:
            self._send_regular_payload_tele()
            rospy.sleep(self._tele_interval)
        elif self.link_select == 1:
            self._send_regular_payload_sms()
            rospy.sleep(self._sms_interval)
        else:
            self._send_regular_payload_sbd()
            # The sleep interval is controlled by sbd link node

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("mavros/state", State, self.payloads.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.payloads.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.payloads.get_GPS_coord)
        rospy.Subscriber("mavros/rc/out", RCOut, self.payloads.get_VTOL_mode)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.payloads.get_wp_reached)
        rospy.Subscriber("ogc/statustext", YonahStatusText, self.get_status_text)
        rospy.Subscriber("mavros/vibration/raw/vibration", Vibration, self.payloads.get_vibe_status)
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        alerts = rospy.Timer(rospy.Duration(1), self.check_alerts)
        message_sender = rospy.Timer(rospy.Duration(1), self.send_regular_payload)
        rospy.spin()
        alerts.shutdown()
        message_sender.shutdown()

if __name__=='__main__':
    run = airdespatcher()
    run.client()

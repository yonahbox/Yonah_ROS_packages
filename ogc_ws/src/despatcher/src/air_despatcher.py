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

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import Vibration
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from std_msgs.msg import String
from statustext.msg import YonahStatusText
from despatcher.msg import Message

# Local
from regular import air_payload

class airdespatcher():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('air_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.pub_to_telegram = rospy.Publisher('ogc/to_telegram', Message, queue_size = 5) # Link to telegram node      
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', String, queue_size = 5) # Link to SBD node

        # Msg handlers
        self._msg = "" # Stores outgoing msg Air-to-Ground message
        self._recv_msg = list() # Stores incoming Ground-to-Air message in list form
        self._regular_payload_flag = True # Whether we should send regular payload to Ground Control
        self._statustext_flag = True # Whether we should send status texts to Ground Control
        self.payloads = air_payload() # Handler for regular and on-demand payloads
        self.link_select = 0 # 0 = Tele, 1 = SMS, 2 = SBD

        # Temp params for aircraft/gnd identifiers and prefix
        # Severity lvl not declared as classwide param as it changes constantly (we want to avoid race conditions!)
        # To-do: Work on air/gnd identifiers whitelist file
        self._is_air = 1 # 1 if aircraft, 0 if GCS (outgoing msg)
        self._id = 1 # ID number (outgoing msg)
        self._prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous recv msg (incoming msg)

        # Intervals btwn msgs
        self._interval_1 = rospy.get_param("~interval_1") # Short time interval (seconds) for regular payload
        self._interval_2 = rospy.get_param("~interval_2") # Long time interval (seconds) for regular payload
        self._sms_interval = self._interval_2 # Interval (seconds) for regular payload over sms
        self._tele_interval = self._interval_2 # Interval (seconds) for regular payload over telegram

        self.ground_id = rospy.get_param("~ground_ids")[0]

        # Wait for MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')

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

    def _check_sender_id(self, is_air, id):
        '''Check identity of the sender'''
        if is_air:
            # We got no business with other aircraft... yet
            return False
        # To-do: Add in checks for id (maybe a whitelist, compare the IMEI/phone numbers?)
        return True
    
    def _check_ping(self):
        '''Check for ping commands from Ground Control'''
        if len(self._recv_msg) == 1: # Simple "ping" request
            self._msg = self.payloads.truncate_regular_payload()
            severity = "r"
        else:
            # Make sure the ping command is of the correct format ("ping <command>")
            if len(self._recv_msg) == 2 and self._recv_msg[0] == 'ping' and \
                self._recv_msg[1] in self.payloads.ping_entries:
                self._msg = str(self.payloads.ping_entries[self._recv_msg[1]])
                severity = "i"
            else:
                return
        self.sendmsg(severity)

    def _check_sms(self):
        '''Check for SMS commands from Ground Control'''
        if self._recv_msg[0] == "sms" and len(self._recv_msg) == 2:
            if self._recv_msg[1] == "true": # Send regular payloads to Ground Control
                self._regular_payload_flag = True
            elif self._recv_msg[1] == "false": # Don't send regular payloads
                self._regular_payload_flag = False
            elif self._recv_msg[1] == "short": # Send regular payloads at short intervals
                self._sms_interval = self._interval_1
            elif self._recv_msg[1] == "long": # Send regular payloads at long intervals
                self._sms_interval = self._interval_2
            else:
                return
        else:
            return
        self._send_ack()

    def _check_statustext(self):
        '''Check for statustext commands from Ground Control'''
        if len(self._recv_msg) == 2 and self._recv_msg[0] == "statustext":
            if self._recv_msg[1] == "true":
                self._statustext_flag = True
            elif self._recv_msg[1] == "false":
                self._statustext_flag = False
            else:
                return
        else:
            return
        self._send_ack()

    def _check_arming(self):
        """Check for Arm/Disarm commands from Ground Control"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        if len(self._recv_msg) == 1:
            if self._recv_msg[0] == "disarm":
                arm(0)
            elif self._recv_msg[0] == "arm":
                arm(1)
            else:
                return
        else:
            return
        self._send_ack()

    def _check_mode(self):
        """Check for Mode change commands from Ground Control"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        # Message structure: mode <flight mode>; extract 2nd word to get flightmode
        if self._recv_msg[0] == 'mode' and len(self._recv_msg) == 2:
            # Set flight mode, check if successful
            if mode(custom_mode = self._recv_msg[1]).mode_sent == True:
                self._send_ack()

    def _check_mission(self):
        """Check for mission/waypoint commands from Ground Control"""
        if self._recv_msg[0] == "wp" and len(self._recv_msg) == 3:
            if self._recv_msg[1] == 'set':
                # Message structure: wp set <seq_no>, extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = self._recv_msg[2]
                # Set target waypoint, check if successful
                if wp_set(wp_seq = int(seq_no)).success == True:
                    self._send_ack()
            elif self._recv_msg[1] == 'load':            
                # Message structure: wp load <wp file name>; extract 3rd word to get wp file
                # Assume that wp file is located in root directory of Beaglebone
                # To do: Switch to WaypointPush service; currently no way to determine whether WPs successfully loaded
                wp_file = self._recv_msg[2]
                subprocess.call(["rosrun", "mavros", "mavwp", "load", "/home/ubuntu/%s"%(wp_file)], shell=False)
                self._send_ack()
            else:
                return

    def check_incoming_msgs(self, data):
        '''Check for incoming G2A messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            rospy.loginfo("Received \"" + data.data + "\"")
            # Handle msg headers, check time + id, and strip out headers
            self._recv_msg = data.data.split()
            sender_timestamp = int(self._recv_msg[-1])
            sender_msgtype = str(self._recv_msg[0])
            sender_is_air = int(self._recv_msg[1])
            sender_id = int(self._recv_msg[2])
            if not self._is_new_msg(sender_timestamp) or not self._check_sender_id(sender_is_air, sender_id):
                return
            if not sender_msgtype == 'i':
                # for now, we only accept info level commands
                return
            self._recv_msg = self._recv_msg[3:-1]
            # Go through series of checks
            if "ping" in self._recv_msg:
                self._check_ping()
            elif "sms" in self._recv_msg:
                self._check_sms()
            elif "statustext" in self._recv_msg:
                self._check_statustext()
            elif "arm" in self._recv_msg or "disarm" in self._recv_msg:
                self._check_arming()
            elif "mode" in self._recv_msg:
                self._check_mode()
            elif "wp" in self._recv_msg:
                self._check_mission()
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
        self.pub_to_sms.publish(self._msg)
    
    def _send_regular_payload_sbd(self):
        '''Send regular payload over SBD Satcomms link'''
        self.pub_to_sbd.publish(self._msg)

    def _send_regular_payload_tele(self):
        '''Send regular payload over Telegram link'''
        print(self.ground_id)
        message = Message()
        message.data = self._msg
        # message.id = 1
        message.id = self.ground_id
        self.pub_to_telegram.publish(message)
    
    def sendmsg(self, severity):
        '''Send any msg that's not a regular payload'''
        self._attach_headers(severity)
        if self.link_select == 0:
            self.pub_to_telegram.publish(self._msg)
        elif self.link_select == 1:
            self.pub_to_sms.publish(self._msg)
        else:
            self.pub_to_sbd.publish(self._msg)
        

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
        elif self.link_select == 1:
            self._send_regular_payload_sms()
        else:
            self._send_regular_payload_sbd()
        
        # Sleep for the specified interval. Note that rospy.Timer
        # will not allow the time interval to go below min_interval
        rospy.sleep(self._sms_interval)

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("mavros/state", State, self.payloads.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.payloads.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.payloads.get_GPS_coord)
        rospy.Subscriber("mavros/rc/out", RCOut, self.payloads.get_VTOL_mode)
        rospy.Subscriber("mavros/mission/reached", WaypointReached, self.payloads.get_wp_reached)
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

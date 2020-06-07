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
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', String, queue_size = 5) # Link to SBD node

        # Msg handlers
        self.msg = "" # Stores outgoing msg Air-to-Ground message
        self.recv_msg = list() # Stores incoming Ground-to-Air message in list form
        self.regular_payload_flag = False # Whether we should send regular payload to Ground Control
        self.payloads = air_payload() # Handler for regular and on-demand payloads

        # Temp params for aircraft/gnd identifiers and prefix
        # Severity lvl not declared as classwide param as it changes constantly (we want to avoid race conditions!)
        # To-do: Work on air/gnd identifiers whitelist file
        self.is_air = 1 # 1 if aircraft, 0 if GCS (outgoing msg)
        self.id = 1 # ID number (outgoing msg)
        self.prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous recv msg (incoming msg)

        # Intervals btwn msgs
        self.interval_1 = rospy.get_param("~interval_1") # Short time interval (seconds) for regular payload
        self.interval_2 = rospy.get_param("~interval_2") # Long time interval (seconds) for regular payload
        self.sms_interval = self.interval_2 # Interval (seconds) for regular payload over sms
        
        # Wait for MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################

    def is_new_msg(self, timestamp):
        '''Return true is incoming msg is a new msg'''
        if timestamp < self.prev_transmit_time:
            return False
        else:
            self.prev_transmit_time = timestamp
            return True

    def check_sender_id(self, is_air, id):
        '''Check identity of the sender'''
        if is_air:
            # We got no business with other aircraft... yet
            return False
        # To-do: Add in checks for id (maybe a whitelist, compare the IMEI/phone numbers?)
        return True
    
    def check_incoming_msgs(self, data):
        '''Check for incoming G2A messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            rospy.loginfo("Received \"" + data.data + "\"")
            # Handle msg headers, check time + id, and strip out headers
            self.recv_msg = data.data.split()
            sender_timestamp = int(self.recv_msg[-1])
            sender_msgtype = str(self.recv_msg[0])
            sender_is_air = int(self.recv_msg[1])
            sender_id = int(self.recv_msg[2])
            if not self.is_new_msg(sender_timestamp) or not self.check_sender_id(sender_is_air, sender_id):
                return
            if not sender_msgtype == 'i':
                # for now, we only accept info level commands
                return
            self.recv_msg = self.recv_msg[3:-1]
            # Go through series of checks
            if "ping" in self.recv_msg:
                self.check_ping()
            elif "sms" in self.recv_msg:
                self.check_sms()
            elif "arm" in self.recv_msg or "disarm" in self.recv_msg:
                self.check_arming()
            elif "mode" in self.recv_msg:
                self.check_mode()
            elif "wp" in self.recv_msg:
                self.check_mission()
        except(rospy.ServiceException):
            rospy.logwarn("Service Call Failed")
        except (ValueError, IndexError):
            rospy.logerr("Invalid message format")
    
    def check_ping(self):
        '''Check for ping commands from Ground Control'''
        if len(self.recv_msg) == 1: # Simple "ping" request
            self.msg = self.payloads.truncate_regular_payload()
            severity = "r"
        else:
            # Make sure the ping command is of the correct format ("ping <command>")
            if len(self.recv_msg) == 2 and self.recv_msg[0] == 'ping' and \
                self.recv_msg[1] in self.payloads.ping_entries:
                self.msg = str(self.payloads.ping_entries[self.recv_msg[1]])
                severity = "i"
            else:
                return
        self.sendmsg(severity)

    def check_sms(self):
        '''Check for SMS commands from Ground Control'''
        if self.recv_msg[0] == "sms" and len(self.recv_msg) == 2:
            if self.recv_msg[1] == "true": # Send regular payloads to Ground Control
                self.regular_payload_flag = True
            elif self.recv_msg[1] == "false": # Don't send regular payloads
                self.regular_payload_flag = False
            elif self.recv_msg[1] == "short": # Send regular payloads at short intervals
                self.sms_interval = self.interval_1
            elif self.recv_msg[1] == "long": # Send regular payloads at long intervals
                self.sms_interval = self.interval_2
            else:
                return
        else:
            return
        self.send_ack()

    def check_arming(self):
        """Check for Arm/Disarm commands from Ground Control"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        if len(self.recv_msg) == 1:
            if self.recv_msg[0] == "disarm":
                arm(0)
            elif self.recv_msg[0] == "arm":
                arm(1)
            else:
                return
        else:
            return
        self.send_ack()

    def check_mode(self):
        """Check for Mode change commands from Ground Control"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        # Message structure: mode <flight mode>; extract 2nd word to get flightmode
        if self.recv_msg[0] == 'mode' and len(self.recv_msg) == 2:
            # Set flight mode, check if successful
            if mode(custom_mode = self.recv_msg[1]).mode_sent == True:
                self.send_ack()

    def check_mission(self):
        """Check for mission/waypoint commands from Ground Control"""
        if self.recv_msg[0] == "wp" and len(self.recv_msg) == 3:
            if self.recv_msg[1] == 'set':
                # Message structure: wp set <seq_no>, extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = self.recv_msg[2]
                # Set target waypoint, check if successful
                if wp_set(wp_seq = int(seq_no)).success == True:
                    self.send_ack()
            elif self.recv_msg[1] == 'load':            
                # Message structure: wp load <wp file name>; extract 3rd word to get wp file
                # Assume that wp file is located in root directory of Beaglebone
                # To do: Switch to WaypointPush service; currently no way to determine whether WPs successfully loaded
                wp_file = self.recv_msg[2]
                subprocess.call(["rosrun", "mavros", "mavwp", "load", "/home/ubuntu/%s"%(wp_file)], shell=False)
                self.send_ack()
            else:
                return

    #########################################
    # Handle Air-to-Ground (A2G) messages
    #########################################

    def check_alerts(self, data):
        '''Check for special alerts'''
        # Vibrations. Vibes above 30 are considered bad
        bad_vibes = 30
        previous_vibes = self.payloads.ping_entries["vibe"]
        i = 0
        while i < 3:
            if self.payloads.ping_entries["vibe"][i] >= bad_vibes and previous_vibes[i] < bad_vibes:
                self.msg = "Vibrations are high"
                self.sendmsg("w")
                break
            i = i + 1
        # Add more special alert checks here
        # To-do: Better msg prefix system
    
    def send_ack(self):
        '''Send a acknowledgement of the msg in self.recv_msg'''
        self.msg = ' '.join(self.recv_msg)
        self.sendmsg("a")
    
    def attach_headers(self, severity):
        '''Attach message headers (prefixes and suffixes'''
        self.msg = severity + " " + str(self.is_air) + " " + str(self.id) + " " + \
            self.msg + " " + str(rospy.get_rostime().secs)

    def send_regular_payload_sms(self):
        '''Send regular payload over sms link'''
        self.pub_to_sms.publish(self.msg)
    
    def send_regular_payload_sbd(self):
        '''Send regular payload over SBD Satcomms link'''
        self.pub_to_sbd.publish(self.msg)

    def send_regular_payload_tele(self):
        '''Send regular payload over Telegram link'''
        pass
    
    def send_regular_payload(self, data):
        if self.regular_payload_flag == False:
            return
        '''Send regular payload over one of the three links: SMS, SBD or Telegram'''
        self.msg = self.payloads.truncate_regular_payload()
        self.attach_headers("r")
        self.send_regular_payload_sms() # To-do: Replace with if-else statement
        self.send_regular_payload_sbd()
        # Sleep for the specified interval. Note that rospy.Timer
        # will not allow the time interval to go below min_interval
        rospy.sleep(self.sms_interval)
    
    def sendmsg(self, severity):
        '''Send any msg that's not a regular payload'''
        self.attach_headers(severity)
        self.pub_to_sms.publish(self.msg) # To-do: Replace with if-else statement
        self.pub_to_sbd.publish(self.msg)
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("mavros/state", State, self.payloads.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.payloads.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.payloads.get_GPS_coord)
        rospy.Subscriber("mavros/rc/out", RCOut, self.payloads.get_VTOL_mode)
        rospy.Subscriber("mavros/mission/reached", WaypointReached, self.payloads.get_wp_reached)
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
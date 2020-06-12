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
import csv
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from mavros_msgs.msg import RCOut
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

# Local
from regular import air_payload

# Global variables
waypoints = []

class WP(object):

    # Creates a csv dialect to read the waypoint file properly
    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def read(self, wpfile):
        f = open(wpfile, "r")
        # There are header lines which identify waypoint files
        pastheaderline = False
        for data in csv.reader(f, self.CSVDialect):
            if not pastheaderline:
                qgc, wpl, ver = data[0].split(' ', 3)
                ver = int(ver)
                if qgc == 'QGC' and wpl == 'WPL' and (ver == 110 or ver ==120):
                    pastheaderline = True

            else:
                # Convert waypoints into Waypoint format
                waypoints.append(Waypoint(
                    is_current = bool(int(data[1])),
                    frame = int(data[2]),
                    command = int(data[3]),
                    param1 = float(data[4]),
                    param2 = float(data[5]),
                    param3 = float(data[6]),
                    param4 = float(data[7]),
                    x_lat = float(data[8]),
                    y_long = float(data[9]),
                    z_alt = float(data[10]),
                    autocontinue = bool(int(data[11]))
                ))

class airdespatcher():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('air_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.pub_to_telegram = rospy.Publisher('ogc/to_telegram', String, queue_size = 5) # Link to telegram node      
        self.pub_to_sbd = rospy.Publisher('ogc/to_sbd', String, queue_size = 5) # Link to SBD node

        # Msg handlers
        self._msg = "" # Stores outgoing msg Air-to-Ground message
        self._recv_msg = list() # Stores incoming Ground-to-Air message in list form
        self._regular_payload_flag = False # Whether we should send regular payload to Ground Control
        self._statustext_flag = False # Whether we should send status texts to Ground Control
        self.payloads = air_payload() # Handler for regular and on-demand payloads

        # Temp params for aircraft/gnd identifiers and prefix
        # Severity lvl not declared as classwide param as it changes constantly (we want to avoid race conditions!)
        # To-do: Work on air/gnd identifiers whitelist file
        self._is_air = 1 # 1 if aircraft, 0 if GCS (outgoing msg)
        self._id = 1 # ID number (outgoing msg)
        self._prev_transmit_time = rospy.get_rostime().secs # Transmit time of previous recv msg (incoming msg)

        # Intervals btwn msgs
        self._interval_1 = rospy.get_param("~_interval_1") # Short time interval (seconds) for regular payload
        self._interval_2 = rospy.get_param("~_interval_2") # Long time interval (seconds) for regular payload
        self._sms_interval = self._interval_2 # Interval (seconds) for regular payload over sms
        self._tele_interval = self._interval_2 # Interval (seconds) for regular payload over telegram

        # Wait for MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')
        rospy.wait_for_service('mavros/mission/push')

        self.hop = False
        self.missionlist = []

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
        global waypoints
        wpfolder = rospy.get_param('~waypoint_folder', '/home/ubuntu/Yonah_ROS_packages/Waypoints/')
        # wpfolder = "/home/huachen/Yonah/Yonah_ROS_packages/Waypoints/"
        """Check for mission/waypoint commands from Ground Control"""
        if self._recv_msg[0] == "wp":
            if self._recv_msg[1] == 'set':
                # Message structure: wp set <seq_no>, extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = self._recv_msg[2]
                # Set target waypoint, check if successful
                if wp_set(wp_seq = int(seq_no)).success == True:
                    self._send_ack()
            elif self._recv_msg[1] == 'load':
                # Message structure: wp load <wp file name.txt>; extract 3rd word to get wp file
                # Assume that wp file is located in Waypoints folder of Beaglebone
                wp_file = self._recv_msg[2]
                # Set to non-hop mission
                self.hop = False
                # Reset mission and waypoints list
                self.missionlist = []
                waypoints = []
                readwp = WP()
                try:
                    readwp.read(str(wpfolder + wp_file))
                    wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
                    # Push waypoints, check if successful
                    if wp(0, waypoints).success:
                        self._send_ack()
                except FileNotFoundError:
                    self._msg  = "Specified file not found"
                    self.sendmsg("e")
            else:
                return
        if self._recv_msg[0] == "mission":
            if self._recv_msg[1] == 'load':
                # Message structure: mission load <mission file name.txt>
                self.missionlist = []
                # Change to hop-mission mode
                self.hop = True
                mission_file = self._recv_msg[2]
                try:
                    f = open(str(wpfolder + mission_file), "r")
                except FileNotFoundError:
                    self._msg = "Specified file not found"
                    self.sendmsg("e")
                    return
                for line in f:
                    # Ignores # comments
                    if line.startswith('#'):
                        continue
                    try:
                        g = open(str(wpfolder + line.rstrip()), "r") # Open and close to check each wp file
                        g.close()
                    except FileNotFoundError:
                        # Specify which file in the list is not found
                        self._msg = str(line.rstrip() + "-->File not found")
                        self.sendmsg("e")
                        # Set to non-hop (in this case it is used as a switch for the next step)
                        # Could not think of another way to elegantly implement this
                        self.hop = False
                f.close()
                # Returns if any of the files in mission list weren't found
                if not self.hop:
                    return
                # Somehow there is a need to open the file again after the try block finishes
                f = open(str(wpfolder + mission_file), "r")
                for line in f:
                    if line.startswith('#'):
                        continue
                    self.missionlist.append(line.rstrip())
                f.close()
                # Prints the missions for operator to check
                self._msg = "Missions: " + " ".join(self.missionlist)
                self.sendmsg("i")
                # Load first mission
                self.current_mission = 0
                readwp = WP()
                readwp.read(str(wpfolder + self.missionlist[0]))
                wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
                if wp(0, waypoints).success:
                    # This acknowledgement implies that the mission list has no errors and the first mission is loaded
                    self._send_ack()

            elif self._recv_msg[1] == 'next':
                # Message structure: wp next (no arguments)
                if not self.hop: # Checks if hop mission
                    self._msg = "Please load a mission file"
                    self.sendmsg("e")
                    return
                self.current_mission += 1
                if self.current_mission >= len(self.missionlist):
                    self.hop = False
                    self.current_mission = 0
                    self._msg = "There are no more missions"
                    self.sendmsg("e")
                    return
                waypoints = []
                readwp = WP()
                try:
                    readwp.read(str(wpfolder + self.missionlist[self.current_mission]))
                    wp = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
                    if wp(0, waypoints).success:
                        self._send_ack()
                # This error should never be raised if everything above works
                except FileNotFoundError:
                    self._msg = "Specified file not found"
                    self.sendmsg("e")
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
            elif "wp" in self._recv_msg or "mission" in self._recv_msg:
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
        self.pub_to_telegram.publish(self._msg)
        rospy.sleep(self._tele_interval) # Need to remove this after the merge
    
    def sendmsg(self, severity):
        '''Send any msg that's not a regular payload'''
        self._attach_headers(severity)
        self.pub_to_sms.publish(self._msg) # To-do: Replace with if-else statement
        self.pub_to_sbd.publish(self._msg)
        self.pub_to_telegram.publish(self._msg)
        print(self._msg)

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
        # To-do: Better msg prefix system
    
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
        self._send_regular_payload_sms() # To-do: Replace with if-else statement
        self._send_regular_payload_sbd()
        self._send_regular_payload_tele()
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

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
from time import sleep

# ROS/Third-Party
import rospy
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from mavros_msgs.msg import RCOut
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import StatusText
from mavros_msgs.msg import Vibration
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from std_msgs.msg import String

class airdespatcher():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('air_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.msg = "" # Stores outgoing msg Air-to-Ground message
        self.recv_msg = "" # Stores incoming Ground-to-Air message
        self.regular_payload_flag = False # Whether we should send regular payload to Ground Control
        self.statustext_flag = False # Whether we should send status texts to Ground Control
        self.interval_1 = rospy.get_param("~interval_1") # Short time interval (seconds) for regular payload
        self.interval_2 = rospy.get_param("~interval_2") # Long time interval (seconds) for regular payload
        self.sms_interval = self.interval_2 # Interval (seconds) for regular payload over sms
        self.min_interval = 1 # Minimum allowable time interval (seconds) for regular payload
        self.ack = "ACK: " # Acknowledgement prefix
        self.entries = { # Dictionary to hold all regular payload entries
            "airspeed": 0.0,
            "alt": 0.0,
            "arm": 0,
            "groundspeed": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "throttle": 0.0,
            "wp": 0,
            "vtol": 0,
        }
        self.ping_entries = { # Dictionary to hold all on-demand payload entries
            "mode": "MANUAL",
            "msg": "",
            "vibe": (0.0,0.0,0.0),
            "clipping": (0,0,0),
        }
        # Wait for MAVROS services
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')

    
    ########################################
    # Subsrcription to MAVROS topics
    ########################################
    
    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros/state'''
        self.entries["arm"] = int(data.armed)
        self.ping_entries["mode"] = data.mode
    
    def get_VFR_HUD_data(self, data):
        '''Obtain VFR_HUD data from mavros/vfr_hud'''
        self.entries["airspeed"] = round(data.airspeed, 1)
        self.entries["groundspeed"] = round(data.groundspeed, 1)
        self.entries["throttle"] = round(data.throttle, 1)
        self.entries["alt"] = round(data.altitude, 1)

    def get_GPS_coord(self, data):
        '''Obtain GPS latitude and longitude from mavros/global_position/global'''
        self.entries["lat"] = round(data.latitude, 6)
        self.entries["lon"] = round(data.longitude, 6)

    def get_wp_reached(self, data):
        '''Obtain information on which waypoint has been reached'''
        self.entries["wp"] = data.wp_seq

    def get_VTOL_mode(self, data):
        '''Check whether any of the quad outputs are active, to determine if we are in VTOL mode'''
        if data.channels[4] > 1200 or data.channels[5] > 1200 or data.channels[6] > 1200\
            or data.channels[7] > 1200:
            self.entries["vtol"] = 1
        else:
            self.entries["vtol"] = 0

    def get_status_text(self, data):
        '''Obtain status text messages from mavros/statustext/recv)'''
        self.ping_entries["msg"] = data.text
        # Send new status texts to Ground Control
        if self.statustext_flag == True:
            self.msg = self.ping_entries["msg"]
            self.sendmsg()

    def get_vibration_status(self, data):
        '''Obtain vibration data from mavros/vibration/raw/vibration'''
        bad_vibes = 30 # Vibrations above this level (m/s/s) are considered bad
        previous_vibes = self.ping_entries["vibe"]
        self.ping_entries["vibe"] = (round(data.vibration.x, 2), round(data.vibration.y, 2),\
            round(data.vibration.z, 2))
        self.ping_entries["clipping"] = (data.clipping[0], data.clipping[1], data.clipping[2])
        # Check for special events
        i = 0
        while i < 3:
            if self.ping_entries["vibe"][i] >= bad_vibes and previous_vibes[i] < bad_vibes:
                self.msg = "Warning: Vibrations are high"
                self.sendmsg()
                break
            i = i + 1

    ###########################################
    # Handle Ground-to-Air (G2A) messages
    ###########################################

    def check_incoming_msgs(self, data):
        '''Check for incoming G2A messages from ogc/from_sms, from_sbd or from_telegram topics'''
        try:
            self.recv_msg = data.data
            if "ping" in self.recv_msg:
                self.check_ping()
            elif "sms" in self.recv_msg:
                self.check_sms()
            elif "statustext" in self.recv_msg:
                self.check_statustext()
            elif "arm" in self.recv_msg:
                self.check_arming()
            elif "mode" in self.recv_msg:
                self.check_mode()
            elif "wp" in self.recv_msg:
                self.check_mission()
        except(rospy.ServiceException):
            rospy.logwarn("Service Call Failed")
    
    def check_ping(self):
        '''Check for ping commands from Ground Control'''
        if self.recv_msg == "ping": # Simple "ping" request
            self.truncate_regular_payload()
        else:
            breakdown = self.recv_msg.split()
            # Make sure the ping command is of the correct format ("ping <command>")
            if len(breakdown) == 2 and breakdown[0] == 'ping' and \
                breakdown[1] in self.ping_entries:
                self.msg = str(self.ping_entries[breakdown[1]])
            else:
                return
        self.sendmsg()

    def check_sms(self):
        '''Check for SMS commands from Ground Control'''
        if self.recv_msg == "sms true": # Send regular payloads to Ground Control
            self.regular_payload_flag = True
        elif self.recv_msg == "sms false": # Don't send regular payloads
            self.regular_payload_flag = False
        elif self.recv_msg == "sms short": # Send regular payloads at short intervals
            self.sms_interval = self.interval_1
        elif self.recv_msg == "sms long": # Send regular payloads at long intervals
            self.sms_interval = self.interval_2
        else:
            return
        self.msg = self.ack + self.recv_msg
        self.sendmsg()

    def check_statustext(self):
        '''Check for statustext commands from Ground Control'''
        if self.recv_msg == "statustext true":
            self.statustext_flag = True
        elif self.recv_msg == "statustext false":
            self.statustext_flag = False
        else:
            return
        self.msg = self.ack + self.recv_msg
        self.sendmsg()
    
    def check_arming(self):
        """Check for Arm/Disarm commands from Ground Control"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        if self.recv_msg == "disarm":
            arm(0)
        elif self.recv_msg == "arm":
            arm(1)
        else:
            return
        self.msg = self.ack + self.recv_msg
        self.sendmsg()

    def check_mode(self):
        """Check for Mode change commands from Ground Control"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        mode_breakdown = self.recv_msg.split()
        # Message structure: mode <flight mode>; extract 2nd word to get flightmode
        if mode_breakdown[0] == 'mode' and len(mode_breakdown) == 2:
            # Set flight mode, check if successful
            if mode(custom_mode = mode_breakdown[1]).mode_sent == True:
                self.msg = self.ack + self.recv_msg
                self.sendmsg()

    def check_mission(self):
        """Check for mission/waypoint commands from Ground Control"""
        wp_breakdown = self.recv_msg.split()
        if wp_breakdown[0] == "wp" and len(wp_breakdown) == 3:
            if wp_breakdown[1] == 'set':
                # Message structure: wp set <seq_no>, extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = wp_breakdown[2]
                # Set target waypoint, check if successful
                if wp_set(wp_seq = int(seq_no)).success == True:
                    self.msg = self.ack + self.recv_msg
                    self.sendmsg()
            elif wp_breakdown[1] == 'load':            
                # Message structure: wp load <wp file name>; extract 3rd word to get wp file
                # Assume that wp file is located in root directory of Beaglebone
                # To do: Switch to WaypointPush service; currently no way to determine whether WPs successfully loaded
                wp_file = wp_breakdown[2]
                subprocess.call(["rosrun", "mavros", "mavwp", "load", "/home/ubuntu/%s"%(wp_file)], shell=False)
                self.msg = self.ack + self.recv_msg
                self.sendmsg()
            else:
                return

    #########################################
    # Handle Air-to-Ground (A2G) messages
    #########################################

    def truncate_regular_payload(self):
        '''Remove unnecessary characters from regular payload'''
        self.msg = str(sorted(self.entries.items())) # Sort entries and convert to string
        bad_char = ",[]()'"
        for i in bad_char:
            self.msg = self.msg.replace(i,"") # Remove unnecessary characters
        for k in self.entries.keys():
            k = k + " "
            self.msg = self.msg.replace(k,"") # Remove entry descriptions
    
    def send_regular_payload_sms(self):
        '''Send regular payload over sms link'''
        self.pub_to_sms.publish(self.msg)
        # Sleep for the specified interval. Note that rospy.Timer
        # will not allow the time interval to go below min_interval
        sleep(self.sms_interval)
    
    def send_regular_payload_sbd(self):
        '''Send regular payload over SBD Satcomms link'''
        pass

    def send_regular_payload_tele(self):
        '''Send regular payload over Telegram link'''
        pass
    
    def send_regular_payload(self, data):
        if self.regular_payload_flag == False:
            return
        '''Send regular payload over one of the three links: SMS, SBD or Telegram'''
        self.truncate_regular_payload()
        #rospy.loginfo(self.msg)
        self.send_regular_payload_sms() # To-do: Replace with if-else statement
    
    def sendmsg(self):
        '''Send any msg that's not a regular payload'''
        self.pub_to_sms.publish(self.msg) # To-do: Replace with if-else statement
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.get_GPS_coord)
        rospy.Subscriber("mavros/rc/out", RCOut, self.get_VTOL_mode)
        rospy.Subscriber("mavros/mission/reached", WaypointReached, self.get_wp_reached)
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.get_status_text)
        rospy.Subscriber("mavros/vibration/raw/vibration", Vibration, self.get_vibration_status)
        rospy.Subscriber("ogc/from_sms", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_sbd", String, self.check_incoming_msgs)
        rospy.Subscriber("ogc/from_telegram", String, self.check_incoming_msgs)
        message_sender = rospy.Timer(rospy.Duration(self.min_interval), self.send_regular_payload)
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = airdespatcher()
    run.client()
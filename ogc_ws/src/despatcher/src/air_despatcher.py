#!/usr/bin/env python3

'''
air_despatcher

Bridge between MAVROS and the Ops Ground Control Links (Telegram, SMS, SBD)

'''

# Standard Library
import datetime
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
from std_msgs.msg import String

class airdespatcher():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('air_despatcher', anonymous=False)
        self.pub_to_sms = rospy.Publisher('ogc/to_sms', String, queue_size = 5) # Link to SMS node
        self.msg = "" # Stores outgoing msg to Ground Control
        self.rate = rospy.Rate(0.2) # It seems that I can specify whatever we want here; the real rate is determined by sms_interval
        self.regular_payload_flag = False # Whether we should send regular payload to Ground Control
        self.statustext_flag = False # Whether we should send status texts to Ground Control
        self.interval_1 = rospy.get_param("~interval_1") # Short time interval (seconds) for regular payload
        self.interval_2 = rospy.get_param("~interval_2") # Long time interval (seconds) for regular payload
        self.sms_interval = self.interval_2 # Interval (seconds) for regular payload over sms
        self.min_interval = 1 # Minimum allowable time interval (seconds) for regular payload
        self.ack = "ACK: " # Acknowledgement prefix
        self.entries = { # Dictionary to hold all regular payload entries
            "arm": 0,
            "AS": 0.0,
            "GS": 0.0,
            "thr": 0.0,
            "alt": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "wp": 0,
            "VTOL": 0,
        }
        self.ping_entries = { # Dictionary to hold all on-demand payload entries
            "mode": "MANUAL",
            "msg": "",
            "vibe": (0.0,0.0,0.0),
            "clipping": (0,0,0),
        }

    
    ########################################
    # Subsrcription to MAVROS topics
    ########################################
    
    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros/state'''
        self.entries["arm"] = data.armed
        self.ping_entries["mode"] = data.mode
    
    def get_VFR_HUD_data(self, data):
        '''Obtain VFR_HUD data from mavros/vfr_hud'''
        self.entries["AS"] = round(data.airspeed, 1)
        self.entries["GS"] = round(data.groundspeed, 1)
        self.entries["thr"] = round(data.throttle, 1)
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
            self.entries["VTOL"] = 1
        else:
            self.entries["VTOL"] = 0

    def get_status_text(self, data):
        previous_msg = self.ping_entries["msg"]
        self.ping_entries["msg"] = data.text
        # Send new status texts to Ground Control
        if self.ping_entries["msg"] != previous_msg and self.statustext_flag == True:
            self.msg = self.ping_entries["msg"]
            self.sendmsg()

    def get_vibration_status(self, data):
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
    
    ########################################
    # Subsrcription to MAVROS services
    ########################################

    #

    ###########################################
    # Interaction with other ROS Services/Nodes
    ###########################################

    def check_SMS_rx_node(self, data):
        if "ping" in data.data:
            self.check_ping(data.data)
        elif "sms" in data.data:
            self.check_sms(data.data)
        elif "statustext" in data.data:
            self.check_statustext(data.data)
        else:
            # Default: Send acknowledgement. To-do: Implement whitelist of msgs from SMS_rx
            self.msg = self.ack + data.data
            self.sendmsg()
    
    def check_ping(self, data):
        if data == "ping": # Simple "ping" request
            self.truncate_regular_payload()
        else:
            breakdown = data.split()
            # Make sure the ping command is of the correct format ("ping <command>")
            if len(breakdown) == 2 and breakdown[0] == 'ping' and \
                breakdown[1] in self.ping_entries:
                self.msg = str(self.ping_entries[breakdown[1]])
            else:
                return
        self.sendmsg()

    def check_sms(self, data):
        if data == "sms true": # Send regular payloads to Ground Control
            self.regular_payload_flag = True
        elif data == "sms false": # Don't send regular payloads
            self.regular_payload_flag = False
        elif data == "sms short": # Send regular payloads at short intervals
            self.sms_interval = self.interval_1
        elif data == "sms long": # Send regular payloads at long intervals
            self.sms_interval = self.interval_2
        else:
            return
        self.msg = self.ack + data
        self.sendmsg()

    def check_statustext(self, data):
        if data == "statustext true":
            self.statustext_flag = True
        elif data == "statustext false":
            self.statustext_flag = False
        else:
            return
        self.msg = self.ack + data
        self.sendmsg()

    #########################################
    # Handle sending of msg to Ground Control
    #########################################

    def truncate_regular_payload(self):
        '''Remove unnecessary characters (e.g. , and spaces) from regular payload'''
        self.msg = str(sorted(self.entries.items())) # Sort entries and convert to string
        bad_char = ",[]()'"
        for i in bad_char:
            self.msg = self.msg.replace(i,"") # Remove unnecessary characters
    
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
        rospy.Subscriber("sendsms", String, self.check_SMS_rx_node)
        message_sender = rospy.Timer(rospy.Duration(self.min_interval), self.send_regular_payload)
        #self.rate.sleep()
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = airdespatcher()
    run.client()
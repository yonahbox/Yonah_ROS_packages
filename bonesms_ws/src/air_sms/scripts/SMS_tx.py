#!/usr/bin/env python3

'''
SMS_tx

Subscribe to various MAVROS topics, compile them into an SMS message and send it to Ground Control
MAVROS topics: http://wiki.ros.org/mavros

Prerequisite: Please ensure the GCS number (GCS_no) in air.launch and sms_standalone.launch is correct

Breakdown of regular payload data:
    - Arm status (Arm): 1 or 0
    - Airspeed (AS): m/s
    - Gndspeed (GS): m/s
    - Throttle (thr): Scaled between 0.0 and 1.0
    - Altitude relative to home (alt): m
    - Lat (lat)
    - Lon (lon)
    - Status of AWS Data Telemetry Node (AWS): 1 = alive, 0 = dead
    - Waypoint that has been reached (wp): Integer
    - Whether aircraft is in VTOL mode (VTOL): 1 = yes, 0 = no

Breakdown of on-demand payload data:
    - Mode (mode): String
    - Status Text Messages (msg): String
    - Vibration levels (vibe): m/s/s in x, y and z axes
    - Clipping Events (clipping): No units; x, y and z axes
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
from std_msgs.msg import String

# Local
import RuTOS

class SMStx():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('SMS_tx', anonymous=False)
        self.router_hostname = rospy.get_param("~router_hostname","root@192.168.1.1") # Hostname and IP of onboard router
        self.pub_to_data = rospy.Publisher('sms_to_data', String, queue_size = 5) # Publish to air_data node
        self.rate = rospy.Rate(0.2) # It seems that I can specify whatever we want here; the real rate is determined by self.interval
        self.sms_flag = False # Whether we should send regular payload to Ground Control
        self.short_interval = rospy.get_param("~short_interval") # Short time interval (seconds) for regular payload
        self.long_interval = rospy.get_param("~long_interval") # Long time interval (seconds) for regular payload
        self.interval = self.long_interval # Interval (seconds) for regular payload, defaults to long_interval on bootup
        self.min_interval = 1 # Minimum allowable time interval (seconds) for regular payload
        self.GCS_no = rospy.get_param("~GCS_no", "12345678") # GCS phone number
        self.msg = "" # Stores outgoing SMS to Ground Control
        self.entries = { # Dictionary to hold all regular payload entries
            "arm": 0,
            "AS": 0.0,
            "GS": 0.0,
            "thr": 0.0,
            "alt": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "AWS": 0,
            "wp": 0,
            "VTOL": 0,
        }
        self.ping_entries = { # Dictionary to hold all on-demand payload entries
            "mode": "MANUAL",
            "msg": "",
            "vibe": (0.0,0.0,0.0),
            "clipping": (0,0,0),
        }
    
    #####################################
    # Interaction with ROS Services/Nodes
    #####################################
    
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
        '''Obtain special messages from mavros/statustext'''
        self.ping_entries["msg"] = data.text

    def get_vibration_status(self, data):
        '''Obtain vibration and clipping data from mavros/vibration/raw/vibration'''
        self.ping_entries["vibe"] = (round(data.vibration.x, 2), round(data.vibration.y, 2), round(data.vibration.z, 2))
        self.ping_entries["clipping"] = (data.clipping[0], data.clipping[1], data.clipping[2])

    def check_air_data_status(self, data):
        '''
        Check whether connection between air_data node and GCS is alive or dead. 
        If connection is dead, notify GCS (through SMS) on the connection status.
        If connection is alive, air_data will publish "SVC" (serviceable) message
        '''
        # Todo: Implement a system that will take action when no message is received from the data_to_sms topic
        # after a certain value. The rospy.wait_for_message seems to be what we want for this; see
        # https://docs.ros.org/diamondback/api/rospy/html/rospy.client-module.html#wait_for_message
        if data.data == "SVC":
            self.entries["AWS"] = 1
        else:
            self.entries["AWS"] = 0

    def check_SMS_rx_node(self, data):
        '''
        Subscribe to SMS_rx node to see if we should send SMS, whether in long or short intervals,
        or whether to send a message once (ping reply)
        '''
        if data.data == "sms true":
            self.sms_flag = True
            self.msg = "Regular SMS updating activated"
        elif data.data == "sms false":
            self.sms_flag = False
            self.msg = "Regular SMS updating deactivated"
        elif data.data == "sms short":
            self.interval = self.short_interval
            self.msg = "Regular SMS update intervals set to short"
        elif data.data == "sms long":
            self.interval = self.long_interval
            self.msg = "Regular SMS update intervals set to long"
        elif data.data == "ping":
            self.truncate_regular_payload()
        elif "ping" in data.data:
            ping_breakdown = data.data.split()
            # Make sure the ping command is of the correct format ("ping <command>")
            if len(ping_breakdown) == 2 and ping_breakdown[0] == 'ping' and \
                ping_breakdown[1] in self.ping_entries:
                self.msg = self.ping_entries[ping_breakdown[1]]
            else:
                return
        else:
            return
        self.sendmsg()

    #########################################
    # Handle sending of SMS to Ground Control
    #########################################
    
    def send_msg_at_specified_interval(self, data):
        '''
        Send regular SMS payloads (either short or long interval)
        Regular paylod is active only if it is requested by Ground Control (sms_flag = true)
        '''
        if self.sms_flag:
            self.truncate_regular_payload()
            self.sendmsg()
        else:
            rospy.loginfo("SMS sending is deactivated")
            self.pub_to_data.publish("air_sms to GCS: SMS sending is deactivated")
        
        # Sleep for the specified interval. Note that rospy.Timer
        # will not allow the time interval to go below min_interval
        sleep(self.interval)
    
    def truncate_regular_payload(self):
        '''
        Remove unnecessary characters (e.g. , and spaces) from regular payload
        '''
        self.msg = str(sorted(self.entries.items())) # Sort entries and convert to string
        bad_char = ",[]()'"
        for i in bad_char:
            self.msg = self.msg.replace(i,"") # Remove unnecessary characters
    
    def sendmsg(self):
        '''
        Compile info from all mavros topics into a msg string and send it as an SMS.
        Also inform air_data node of outcome, so that air_data can inform Ground Control about air_sms status
        '''
        rospy.loginfo("Sending SMS to Ground Control")
        try:
            sendstatus = RuTOS.send_msg(self.router_hostname, self.GCS_no, self.msg)
            if sendstatus == "Timeout":
                rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
                self.pub_to_data.publish("air_sms: Msg sending Timeout")
            else:
                self.pub_to_data.publish("air_sms: Msg sending success")
        except(subprocess.CalledProcessError):
            rospy.logwarn("SSH process into router has been killed.")
            self.pub_to_data.publish("air_sms: Cannot ssh into air router")
            

    ############################
    # "Main" function
    ############################
    
    def prepare(self):
        '''
        Main function to subscribe to all mavros topics and send SMS messages (if requested by SMS_rx node)
        If regular payload is activated, messages are sent with a time interval (seconds) specified by self.interval
        '''
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.get_GPS_coord)
        rospy.Subscriber("mavros/rc/out", RCOut, self.get_VTOL_mode)
        rospy.Subscriber("mavros/mission/reached", WaypointReached, self.get_wp_reached)
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.get_status_text)
        rospy.Subscriber("mavros/vibration/raw/vibration", Vibration, self.get_vibration_status)
        rospy.Subscriber("data_to_sms", String, self.check_air_data_status)
        rospy.Subscriber("sendsms", String, self.check_SMS_rx_node)
        message_sender = rospy.Timer(rospy.Duration(self.min_interval), self.send_msg_at_specified_interval)
        self.rate.sleep()
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = SMStx()
    run.prepare()
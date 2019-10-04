#!/usr/bin/env python3

'''
Subscribe to various MAVROS topics, compile them into an SMS message and send it to Ground Control
MAVROS topics: http://wiki.ros.org/mavros

Prerequesite: Please ensure the GCS number (GCS_no) in SMS_main.launch is correct

Message breakdown (each entry is separated by a space):
Armed: 1 or 0
Mode: String
Airspeed: m/s
Gndspeed: m/s
Heading: deg
Throttle: Scaled between 0.0 and 1.0
Altitude (relative to home location): m
Climb Rate: m/s
Lat
Lon
AWS (status of AWS Data Telemetry Node): 1 = alive, 0 = dead
'''

import rospy
from time import sleep
from std_msgs.msg import String
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import RCOut
import subprocess

class SMStx():

    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('SMS_tx', anonymous=False)
        self.pub_to_data = rospy.Publisher('sms_to_data', String, queue_size = 5) # Publish stuff to air_data node
        self.rate = rospy.Rate(0.2) # It seems that I can specify whatever we want here; the real rate is determined by self.interval
        self.sms_flag = False # Determine whether we should send SMS to Ground Control
        self.short_interval = rospy.get_param("~short_interval") # Short time interval between each SMS sent (seconds)
        self.long_interval = rospy.get_param("~long_interval") # Long time interval between each SMS sent (seconds)
        self.interval = self.long_interval # Interval between each SMS (in seconds) defaults to long_interval on bootup
        self.min_interval = 1 # Minimum allowable time interval between each SMS sent (1 second)
        self.GCS_no = rospy.get_param("~GCS_no", "12345678") # GCS phone number
        self.entries = { # Dictionary to hold all message entries
            "arm": 0,
            "mode": "MANUAL",
            "AS": 0.0,
            "thr": 0.0,
            "alt": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            "AWS": 0,
            "wp": 0,
            "VTOL": 0,
        }
    
    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros/state'''
        self.entries["arm"] = data.armed
        self.entries["mode"] = data.mode
    
    def get_VFR_HUD_data(self, data):
        '''Obtain VFR_HUD data from mavros/vfr_hud'''
        self.entries["AS"] = round(data.airspeed, 1)
        self.entries["GS"] = round(data.groundspeed, 1)
        self.entries["thr"] = data.throttle
        self.entries["alt"] = round(data.altitude, 1)

    def get_GPS_coord(self, data):
        '''Obtain GPS latitude and longitude from mavros/global_position/global'''
        self.entries["lat"] = round(data.latitude, 6)
        self.entries["lon"] = round(data.longitude, 6)

    def get_wp_reached(self, data):
        '''Obtain information on which waypoint number has been reached'''
        self.entries["wp"] = data.wp_seq

    def get_VTOL_mode(self, data):
        '''Check whether any of the quad outputs are active, to determine if we are in VTOL mode'''
        if data.channels[4] > 1200 or data.channels[5] > 1200 or data.channels[6] > 1200\
            or data.channels[7] > 1200:
            self.entries["VTOL"] = 1
        else:
            self.entries["VTOL"] = 0

    def check_air_data_status(self, data):
        '''
        Check whether connection between air_data node and GCS is alive or dead.. If connection is dead, 
        increase SMS sending frequency, and notify GCS (through SMS) on the connection status.
        If connection is alive, air_data will publish "SVC" (serviceable) message
        '''
        # Todo: Implement a system that will take action when no message is received from the data_to_sms topic
        # after a certain value. The rospy.wait_for_message seems to be what we want for this; see
        # https://docs.ros.org/diamondback/api/rospy/html/rospy.client-module.html#wait_for_message
        if data.data == "SVC":
            self.entries["AWS"] = 1
            self.interval = self.long_interval
        else:
            self.entries["AWS"] = 0
            self.interval = self.short_interval

    def check_SMS(self, data):
        '''
        Subscribe to SMS_rx node to see if we should send SMS, whether in long or short intervals,
        or whether to send a message once (ping reply)
        '''
        if data.data == "sms true":
            self.sms_flag = True
        elif data.data == "sms false":
            self.sms_flag = False
        elif data.data == "sms short":
            self.interval = self.short_interval
        elif data.data == "sms long":
            self.interval = self.long_interval
        elif data.data == "ping":
            self.sendmsg()

    def send_msg_at_specified_interval(self, data):
        '''
        Send SMS at regular intervals (either short or long interval)
        Regular sending is active only if it is requested by Ground Control (sms_flag = true)
        '''
        if self.sms_flag:
            self.sendmsg()
        else:
            rospy.loginfo("SMS sending is deactivated")
            self.pub_to_data.publish("air_sms to GCS: SMS sending is deactivated")
        
        # Sleep for the specified interval. For some reason, we cannot exploit rospy.Duration's capability
        # to control the interval. Thus, the following line is required. Note that rospy.Timer
        # will not allow the time interval to go below the minimum allowable interval (min_interval)
        sleep(self.interval)
    
    def sendmsg(self):
        '''
        Compile info from all mavros topics into a msg string and send it as an SMS.
        Also publish sending result to air_data node, so that air_data can inform Ground Control about status of SMS
        '''
        msg = str(self.entries)
        rospy.loginfo("Sending SMS to Ground Control")
        try:
            sendstatus = subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -s '%s %s'"%(self.GCS_no, msg)], shell=False)
            if sendstatus == "Timeout":
                rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
                self.pub_to_data.publish("air_sms to GCS: Msg sending Timeout")
            else:
                self.pub_to_data.publish("air_sms_to_GCS: Success")
        except(subprocess.CalledProcessError):
            rospy.logwarn("SSH process into router has been killed.")
            self.pub_to_data.publish("air_sms to GCS: Cannot ssh into air router")
            

    def prepare(self):
        '''
        Main function to subscribe to all mavros topics and send SMS message (if requested by SMS_rx node)
        If SMS sending is activated, messages are sent at an interval (in seconds) specified by self.interval
        '''
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.get_GPS_coord)
        rospy.Subscriber("mavros/mission/reached", WaypointReached, self.get_wp_reached)
        rospy.Subscriber("data_to_sms", String, self.check_air_data_status)
        rospy.Subscriber("sendsms", String, self.check_SMS)
        message_sender = rospy.Timer(rospy.Duration(self.min_interval), self.send_msg_at_specified_interval)
        self.rate.sleep()
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = SMStx()
    run.prepare()

#Old Ways
#    cmd2RUT = "echo Mode: %s Arm: %s | nc 192.168.1.1 5002"%(data.mode, data.armed)
#    os.system(cmd2RUT)
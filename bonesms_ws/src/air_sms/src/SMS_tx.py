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
'''
# Current status: VFD_HUD and GPS topics is are not being subscribed to. This is work in progress

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
#from sensor_msgs.msg import Imu
import subprocess

class SMStx():

    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('SMS_tx', anonymous=False)
        self.rate = rospy.Rate(0.2)
        self.sms_flag = False # Determine whether we should send SMS to Ground Control
        self.GCS_no = rospy.get_param("~GCS_no", "12345678") # GCS phone number
        self.entries = { # Dictionary to hold all message entries
            "arm": 0,
            "mode": "MANUAL",
            "arspd": 0.0,
            "gndspd": 0.0,
            "heading": 0.0,
            "thr": 0.0,
            "rel_alt": 0.0,
            "climb": 0.0,
            "lat": 0.0,
            "lon": 0.0,
            #"rll": 0.0,
            #"pit": 0.0,
            #"yaw": 0.0
        }
    
    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros_msgs/State'''
        self.entries["arm"] = data.armed
        self.entries["mode"] = data.mode
    
    def get_VFR_HUD_data(self, data):
        '''Obtain VFR_HUD data (to be displayed eventually on MavP horizon module)'''
        self.entries["arspd"] = data.airspeed
        self.entries["gndspd"] = data.groundspeed
        self.entries["heading"] = data.heading
        self.entries["thr"] = data.throttle
        self.entries["rel_alt"] = data.altitude
        self.entries["climb"] = data.climb

    def get_GPS_coord(self, data):
        '''Obtain GPS latitude and longitude'''
        self.entries["lat"] = data.latitude
        self.entries["lon"] = data.longitude

    #def get_RPY(self, data):
    #    '''Obtain IMU Roll, Pitch and Yaw Angles'''
    #    self.entries["rll"] = data

    def check_sms_true_false(self, data):
        '''Subscribe to SMS_rx node to see if we should send SMS'''
        if data.data == "true":
            self.sms_flag = True
        if data.data == "false":
            self.sms_flag = False

    def sendmsg(self, data):
        '''
        Compile info from all mavros topics into a msg string and send it as an SMS.
        SMS is sent only if message sending is explicitly requested by Ground Control (sms_flag = true)
        '''
        if self.sms_flag:
            msg = str(self.entries)
            rospy.loginfo("Sending SMS to Ground Control")
            try:
                subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -s '%s %s'"%(self.GCS_no, msg)], shell=False)
            except(subprocess.CalledProcessError):
                rospy.logwarn("SSH process into router has been killed.")
        else:
            rospy.loginfo("SMS sending is deactivated")

    def prepare(self):
        '''
        Main function to subscribe to all mavros topics and send SMS message (if requested by SMS_rx node)
        If SMS sending is activated, messages are sent once every 2 seconds (specified by rospy.Timer)
        To do: Design a system to vary the SMS sending interval (e.g. send more frequently when other links are down)
        '''
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.get_GPS_coord)
        #rospy.Subscriber("imu/data", Imu, self.get_RPY)
        rospy.Subscriber("sendsms", String, self.check_sms_true_false)
        message_sender = rospy.Timer(rospy.Duration(2), self.sendmsg)
        self.rate.sleep()
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = SMStx()
    run.prepare()

#Old Ways
#    cmd2RUT = "echo Mode: %s Arm: %s | nc 192.168.1.1 5002"%(data.mode, data.armed)
#    os.system(cmd2RUT)
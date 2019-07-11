#!/usr/bin/env python3

'''
Subscribe to various MAVROS topics, compile them into an SMS message and send it to Ground Control
MAVROS topics: http://wiki.ros.org/mavros

Prerequesite: Please ensure the GCS number (GCS_no) in SMS_tx.launch is correct

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
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
#from sensor_msgs.msg import Imu
import subprocess

class SMStx():

    def __init__(self, GCS_no):
        '''Initialize all message entries'''
        rospy.init_node('SMS_tx', anonymous=False)
        self.rate = rospy.Rate(0.2)
        self.GCS_no = GCS_no # Receiver/Ground Control phone number
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
        #rospy.loginfo(rospy.get_caller_id() + " Nemo is mode: %s armed: %s", data.mode, data.armed)
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
        self.sendmsg()

    #def get_RPY(self, data):
    #    '''Obtain IMU Roll, Pitch and Yaw Angles'''
    #    self.entries["rll"] = data

    def sendmsg(self):
        '''Compile info from all MAVROS topics into a msg string and send it as an SMS'''
        msg = str(self.entries)
        try:
            cmd2RUT = subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -s '%s %s'"%(self.GCS_no, msg)], shell=False)
        except(subprocess.CalledProcessError):
            print("SSH process into router has been killed.")

    def prepare(self):
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.get_VFR_HUD_data)
        rospy.Subscriber("mavros/global_position/global", NavSatFix, self.get_GPS_coord)
        #rospy.Subscriber("imu/data", Imu, self.get_RPY)
        self.rate.sleep()
        rospy.spin()

if __name__=='__main__':
    GCS_no = "12345678"
    run = SMStx(GCS_no)
    run.prepare()

#Old Ways
#    cmd2RUT = "echo Mode: %s Arm: %s | nc 192.168.1.1 5002"%(data.mode, data.armed)
#    os.system(cmd2RUT)
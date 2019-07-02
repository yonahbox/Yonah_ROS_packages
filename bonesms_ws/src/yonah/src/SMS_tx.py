#!/usr/bin/env python3

'''
Subscribe to various MAVROS topics, compile them into an SMS message and send it to Ground Control
MAVROS topics: http://wiki.ros.org/mavros

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

import rospy
import os
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
#from sensor_msgs.msg import Imu
import subprocess

class SMStx():

    def __init__(self, recv_no):
        '''Initialize all message entries'''
        self.recv_no = recv_no # Receiver/Ground Control phone number
        self.entries = { # Dictionary to hold all message entries
            "arm": 0,
            "mode": "MANUAL",
            #"arspd": 0.0,
            #"gndspd": 0.0,
            #"heading": 0.0,
            #"thr": 0.0,
            #"rel_alt": 0.0,
            #"climb": 0.0,
            #"lat": 0.0,
            #"lon": 0.0,
            #"rll": 0.0,
            #"pit": 0.0,
            #"yaw": 0.0
        }
    
    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros_msgs/State'''
        #rospy.loginfo(rospy.get_caller_id() + " Nemo is mode: %s armed: %s", data.mode, data.armed)
        self.entries["arm"] = data.armed
        self.entries["mode"] = data.mode
        self.sendmsg()
    
    def get_VFD_HUD_data(self, data):
        '''Obtain VFD_HUD data (to be displayed eventually on MavP horizon module)'''
        self.entries["arspd"] = data.Airspeed
        self.entries["gndspd"] = data.Gndspeed
        self.entries["heading"] = data.Heading
        self.entries["thr"] = data.Throttle
        self.entries["rel_alt"] = data.Altitude
        self.entries["climb"] = data.Climb

    def get_GPS_coord(self, data):
        '''Obtain GPS latitude and longitude'''
        self.entries["lat"] = data.Lat
        self.entries["lon"] = data.Lon

    #def get_RPY(self, data):
    #    '''Obtain IMU Roll, Pitch and Yaw Angles'''
    #    self.entries["rll"] = data

    def sendmsg(self):
        '''Compile info from all MAVROS topics into a msg string and send it as an SMS'''
        msg = str(self.entries)
        try:
            #RUTLATLON = subprocess.check_output(["ssh", "root@192.168.1.1", "gpsctl -i && gpsctl -x"], shell=False)
            cmd2RUT = subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -s '%s %s'"%(self.recv_no, msg)], shell=False)
        except(subprocess.CalledProcessError):
            print("SSH process into router has been killed.")

    def prepare(self):
        rospy.init_node('SMS_tx', anonymous=False)
        rospy.Subscriber("mavros/state", State, self.get_mode_and_arm_status)
        #rospy.Subscriber("vfr_hud", VFR_HUD, self.get_VFD_HUD_data)
        #rospy.Subscriber("global_position/global", NavSatFix, self.get_GPS_coord)
        #rospy.Subscriber("imu/data", Imu, self.get_RPY)
        rospy.spin()

if __name__=='__main__':
    recv_no = "+6591993191"
    run = SMStx(recv_no)
    run.prepare()

#Old Ways
#    cmd2RUT = "echo Mode: %s Arm: %s | nc 192.168.1.1 5002"%(data.mode, data.armed)
#    os.system(cmd2RUT)


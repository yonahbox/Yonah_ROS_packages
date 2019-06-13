#!/usr/bin/env python

import rospy
import os
from mavros_msgs.msg import State
import subprocess

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " Nemo is mode: %s armed: %s", data.mode, data.armed)
#    RUTLATLON = subprocess.check_output(["ssh", "root@192.168.1.1", "gpsctl -i && gpsctl -x"], shell=False)
#    LON = subprocess.check_output(["ssh", "root@192.168.1.1", "gpsctl -x"], shell=False)

#    Sending SMS to Ground Control
#    cmd2RUT = subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -s '91514030 Nemo2 Lat:%s,Mode:%s,Arm:%s'"%(RUTLANLON, data.mode, data.armed)], shell=False)

def prepare():
    rospy.init_node('SMS_tx', anonymous=False)
    rospy.Subscriber("mavros/state", State, callback)
    rospy.spin()

if __name__=='__main__':
    prepare()

#Old Ways
#    cmd2RUT = "echo Mode: %s Arm: %s | nc 192.168.1.1 5002"%(data.mode, data.armed)
#    os.system(cmd2RUT)


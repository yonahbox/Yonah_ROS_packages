#!/usr/bin/env python3

"""
sbd_link

ROS node to handle Iridium Short-Burst-Data (SBD) telemetry link on both air and ground side.

Lau Yan Han and Yonah, Apr 2020

Released under the GNU GPL version 3 or later
"""

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from std_msgs.msg import String

class satcomms():

    ################################
    # pyRockblock callback functions
    ################################
    
    def rockBlockConnected(self):
        rospy.loginfo("Rockblock Connected")

    def rockBlockDisconnected(self):
        rospy.logerr("Rockblock Disconnected")
    
    #SIGNAL
    def rockBlockSignalUpdate(self,signal):
        rospy.loginfo("Rockblock signal strength: " + str(signal))

    def rockBlockSignalPass(self):
        rospy.loginfo("Rockblock signal strength good")

    def rockBlockSignalFail(self):
        rospy.logwarn("Rockblock signal strength poor")
    
    #MT
    def rockBlockRxStarted(self):
        rospy.loginfo("Rockblock ready to receive msg")

    def rockBlockRxFailed(self):
        rospy.logwarn("Rockblock unable to check for incoming msg")

    def rockBlockRxReceived(self,mtmsn,data):
        rospy.loginfo("Rockblock msg received: " + data)

    def rockBlockRxMessageQueue(self,count):
        rospy.loginfo("Rockblock found " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("Rockblock ready to send msg")

    def rockBlockTxFailed(self):
        rospy.logwarn("Rockblock msg not sent")

    def rockBlockTxSuccess(self,momsn):
        rospy.loginfo("Rockblock msg sent")

    ############################
    # "Main" function
    ############################
    
    def client(self):
        pass

if __name__=='__main__':
    run = satcomms()
    run.client()
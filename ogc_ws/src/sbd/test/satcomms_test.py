#!/usr/bin/env python3

"""
satcomms_test

Script used to test rockBlock.py module

Lau Yan Han and Yonah, Apr 2020

Released under the GNU GPL version 3 or later
"""

# ROS/Third-Party
import rospy
from std_msgs.msg import String

# Local
import rockBlock
from rockBlock import rockBlockProtocol

class satcommstest(rockBlockProtocol):
    
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
    
    def client(self):
        portID = "/dev/ttyUSB0"
        sbdsession = rockBlock.rockBlock(portID, self)
        while True:
            try:
                momsg = input("Enter your MO msg: ")
                sbdsession.sendMessage(momsg)
            except (KeyboardInterrupt):
                print("Shutting down")
                sbdsession.close()
                break;

if __name__=='__main__':
    run = satcommstest()
    run.client()
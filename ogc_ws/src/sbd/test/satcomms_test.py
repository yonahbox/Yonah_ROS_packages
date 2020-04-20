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
    
    def __init__(self):
        rospy.init_node('satcomms_test', anonymous=False)
        self.portID = "/dev/ttyUSB0"
        self.sbdsession = rockBlock.rockBlock(self.portID, self)
        self.mt_count = 0
    
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

    def send_mo_msg(self, data):
        momsg = input("Enter your MO msg: ")
        self.sbdsession.sendMessage(momsg)
    
    def recv_mt_msg(self, data):
        self.mt_count = self.mt_count + 1
        rospy.loginfo("MT msg read attempt: " + str(self.mt_count))
        self.sbdsession.messageCheck()
    
    def client(self):
        message_sender = rospy.Timer(rospy.Duration(0.5), self.send_mo_msg)
        message_receive = rospy.Timer(rospy.Duration(0.5), self.recv_mt_msg)
        rospy.spin()
        message_sender.shutdown()
        message_receive.shutdown()
        self.sbdsession.close()

if __name__=='__main__':
    run = satcommstest()
    run.client()
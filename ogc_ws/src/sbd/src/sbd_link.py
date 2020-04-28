#!/usr/bin/env python3

"""
sbd_link: ROS node to handle Iridium Short-Burst-Data (SBD) telemetry link on both air and ground side.

Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

# Standard Library
import datetime

# ROS/Third-Party
import rospy
from std_msgs.msg import String

# Local
import rockBlock
from rockBlock import rockBlockProtocol

class satcomms(rockBlockProtocol):

    def __init__(self):
        rospy.init_node('sbd_link', anonymous=False)
        self.portID = "/dev/ttyUSB0"
        self.sbdsession = rockBlock.rockBlock(self.portID, self)
        self.count = 0
        self.buffer = ""
        self.buffer_write_time = ""
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)
        self.interval = 0.5 # sleep interval between mailbox checks
        self.client_serial = "19466"

    ################################
    # pyrockBlock callback functions
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
        self.pub_to_despatcher.publish(data)

    def rockBlockRxMessageQueue(self,count):
        rospy.loginfo("Rockblock found " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("Rockblock ready to send msg")

    def rockBlockTxFailed(self, momsg):
        rospy.logwarn("Rockblock msg not sent: " + momsg)

    def rockBlockTxSuccess(self,momsn, momsg):
        rospy.loginfo("Rockblock msg sent: " + momsg)

    ############################
    # MO/MT msg calls
    ############################
    
    def get_mo_msg(self, data):
        '''
        Get MO msg from to_sbd topic and put it in MO buffer
        Note that MO msg will only be sent on next loop of check_sbd_mailbox
        '''
        self.buffer = "RB00" + self.client_serial + data.data
        self.buffer_write_time = datetime.datetime.now()
    
    def check_sbd_mailbox(self, data):
        '''Initiate an SBD mailbox check'''
        self.count = self.count + 1
        rospy.loginfo("Mailbox read attempt: " + str(self.count))
        # Send MO msg only if required
        if self.buffer == "":
            self.sbdsession.messageCheck(" ")
        else:
            mailchk_time = datetime.datetime.now()
            self.sbdsession.messageCheck(self.buffer)
            # Clear buffer if no new MO msgs were received after sending the previous MO msg
            if self.buffer_write_time < mailchk_time:
                self.buffer = ""

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/to_sbd", String, self.get_mo_msg)
        message_handler = rospy.Timer(rospy.Duration(self.interval), self.check_sbd_mailbox)
        rospy.spin()
        message_handler.shutdown()
        self.sbdsession.close()

if __name__=='__main__':
    run = satcomms()
    run.client()
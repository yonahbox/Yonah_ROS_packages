#!/usr/bin/env python3

"""
satcomms_test: Node used to test sbd_link node and rockBlock module

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

# ROS/Third-Party
import rospy
from std_msgs.msg import String

class satcommstest():
    
    def __init__(self):
        rospy.init_node('satcomms_test', anonymous=False)
        self.pub_to_sbd = rospy.Publisher("ogc/to_sbd", String, queue_size = 5)

    def send_mo_msg(self, data):
        momsg = input()
        self.pub_to_sbd.publish(momsg)
    
    def recv_mt_msg(self, data):
        rospy.loginfo("satcomms_test read: " + data.data)
    
    def client(self):
        message_sender = rospy.Timer(rospy.Duration(0.5), self.send_mo_msg)
        rospy.Subscriber("ogc/from_sbd", String, self.recv_mt_msg)
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = satcommstest()
    run.client()
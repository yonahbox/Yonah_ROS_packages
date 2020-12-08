#!/usr/bin/env python3

'''
intervalchange: Change time interval of an aircraft id and publish it to commstopic. Used in conjunction with airsim.py
'''

# Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rospy
from std_msgs.msg import String

class listener():
    
    def __init__(self):
        self.pub_to_airsim = rospy.Publisher('commstopic', String, queue_size = 5)
        rospy.init_node('intervalchange', anonymous=False)

    def change_interval(self, data):
        interval = input()
        self.pub_to_airsim.publish(interval)

    def client(self):
        self.interval_handler = rospy.Timer(rospy.Duration(0.5), self.change_interval)
        rospy.spin()
        self.interval_handler.shutdown

if __name__=='__main__':
    run = listener()
    run.client()
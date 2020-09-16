#!/usr/bin/env python3

'''
switcher.py: Handle intelligent link-switching logic
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
from std_msgs.msg import UInt8, String

TELE = 0
SMS = 1
SBD = 2

# @TODO: Add logic for sending pings from G2A
# Add logic for sending pings at lower intervals on upstream links

class switcher():

    def __init__(self):
        rospy.init_node('switcher', anonymous=False)
        self.pub_to_despatcher = rospy.Publisher('ogc/from_switcher', UInt8, queue_size=5)
        self._link = TELE
        self._max_time = [0,0,0] # Starting time in seconds
        self._max_time[TELE] = 10
        self._max_time[SMS] = 20
        self._watchdog = list(self._max_time) # Watchdog timer. When timer expires, link switch will trigger
    
    ###########################
    # Watchdog handlers
    ###########################
    
    def _reset_watchdog(self, link):
        self._watchdog[link] = self._max_time[link]
    
    def _switch(self, target_link):
        '''Perform the link-switch action and notify despatcher'''
        self._link = target_link
        self._watchdog[target_link] = self._max_time[target_link]
        rospy.logwarn("Switching to link " + str(target_link))
        self.pub_to_despatcher.publish(self._link)
    
    def countdown(self, data):
        '''Decrement the watchdog by 1 second. When watchdog expires, trigger the link switch'''
        if self._link >= SBD:
            return
        self._watchdog[self._link] = self._watchdog[self._link] - 1
        if self._watchdog[self._link] <= 0:
            self._switch(self._link + 1)

    ###########################
    # Link Monitors
    ###########################
    
    def _monitor_common(self, link):
        # If active link is downstream of current link, trigger link switch to current link
        # Otherwise, simply reset the watchdog of the current link
        if self._link > link:
            self._switch(link)
        else:
            self._reset_watchdog(link)
    
    def monitor_tele(self, data):
        '''Monitor telegram link for incoming msgs'''
        # Add optiimization methods here
        self._monitor_common(TELE)

    def monitor_sms(self, data):
        '''Monitor sms link for incoming msgs'''
        # Add optiimization methods here
        self._monitor_common(SMS)
    
    ###########################
    # "Main" function
    ###########################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.monitor_sms)
        rospy.Subscriber("ogc/from_telegram", String, self.monitor_tele)
        watchdog = rospy.Timer(rospy.Duration(1), self.countdown)
        rospy.spin()
        watchdog.shutdown()

if __name__=='__main__':
    run = switcher()
    run.client()
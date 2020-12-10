#!/usr/bin/env python3

'''
airsim: Sandbox node for testing link switch on multiple aircraft. Used in conjunction with intervalchange.py
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

TELE = 0
SMS = 1
SBD = 2

class aircraft():
    
    def __init__(self, id):
        self.id = id
        self.link = TELE # 0 = Tele, 1 = SMS, 2 = SBD
        self.tele_interval = 2
        self.sms_interval = 5
        self.sbd_interval = 10
        rospy.loginfo("Created Aircraft " + str(self.id))
    
    def change_link(self, link):
        if link == SBD:
            self.tele_interval = 20
            self.sms_interval = 20
            self.sbd_sender = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat_sbd)
        elif link == SMS:
            self.tele_interval = 10
            self.sms_sender = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat_sms)
            try:
                self.sbd_sender.shutdown()
            except:
                pass
        elif link == TELE:
            self.tele_interval = 2
            try:
                self.sms_sender.shutdown()
                self.sbd_sender.shutdown()
            except:
                pass
    
    def send_heartbeat_tele(self, data):
        rospy.loginfo("Tele is alive " + str(self.id))
        rospy.sleep(self.tele_interval)
    
    def send_heartbeat_sms(self, data):
        rospy.loginfo("SMS is alive " + str(self.id))
        rospy.sleep(self.sms_interval)
    
    def send_heartbeat_sbd(self, data):
        rospy.loginfo("SBD is alive " + str(self.id))
        rospy.sleep(self.sbd_interval)
    
    def main(self):
        self.tele_sender = rospy.Timer(rospy.Duration(0.5), self.send_heartbeat_tele)

class talker():
    
    def __init__(self):
        rospy.init_node('airsim', anonymous=False)
        self._pub_to_listener = rospy.Publisher('commstopic', String, queue_size = 5)
        self._valid_ids = rospy.get_param("~valid_ids")
        self.aircrafts = dict()
        self.populate_aircrafts()

    def populate_aircrafts(self):
        for i in self._valid_ids:
            self.aircrafts[i] = aircraft(i)
    
    def monitor_topic(self, data):
        # Msg format: aircraft id + link no
        try:
            msglist = data.data.split()
            id = int(msglist[0])
            link = int(msglist[1])
            self.aircrafts[id].change_link(link)
        except:
            rospy.logerr("Invalid msg")

    def client(self):
        rospy.Subscriber('commstopic', String, self.monitor_topic)
        for i in self._valid_ids:
            self.aircrafts[i].main()
        rospy.spin()

if __name__=='__main__':
    run = talker()
    run.client()
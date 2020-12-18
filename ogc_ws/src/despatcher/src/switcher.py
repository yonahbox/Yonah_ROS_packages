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
import RuTOS
from std_msgs.msg import String, UInt8MultiArray

import headers
from identifiers.srv import GetIds

TELE = 0
SMS = 1
SBD = 2

SMS_timedout = False

class watchdog():
    '''An instance of watchdog should be created for each client'''
    def __init__(self, air_id):
        self.pub_to_despatcher = rospy.Publisher('ogc/from_switcher', String, queue_size=5)
        self._air_id = air_id
        self._link = TELE
        self._max_time = [0,0,0] # Starting time in seconds
        self._max_time[TELE] = 20
        self._max_time[SMS] = 20
        self._watchdog = list(self._max_time) # Watchdog timer. When timer expires, link switch will trigger
        self.countdown_handler = rospy.Timer(rospy.Duration(0.5), self.countdown)

    def reset_watchdog(self, link):
        self._watchdog[link] = self._max_time[link]
    
    def switch(self, target_link):
        '''Perform the link-switch action and notify despatcher'''
        if target_link == 1 and SMS_timedout:
            target_link = 2
        self._link = target_link
        self._watchdog[target_link] = self._max_time[target_link]
        rospy.logwarn("Aircraft " + str(self._air_id) +  " switching to link " + str(target_link))
        self.pub_to_despatcher.publish(str(self._air_id) + " " + str(self._link))
    
    def countdown(self, data):
        '''Decrement the watchdog by 1 second. When watchdog expires, trigger the link switch'''
        if self._link >= SBD:
            return
        self._watchdog[self._link] = self._watchdog[self._link] - 1
        if self._watchdog[self._link] <= 0:
            rospy.logwarn("Watchdog expired, switching")
            self.switch(self._link + 1)
    
    def link_status(self):
        return self._link

    def kill_timers(self):
        self.countdown_handler.shutdown()

class switcher():

    def __init__(self):
        rospy.init_node('switcher', anonymous=False)
        self.pub_to_despatcher = rospy.Publisher('ogc/from_switcher', String, queue_size=5)
        rospy.wait_for_service("identifiers/get/valid_ids")
        ids_get_valid_ids = rospy.ServiceProxy("identifiers/get/valid_ids", GetIds)
        self._valid_ids = ids_get_valid_ids().ids
        self._username = rospy.get_param("~router_username","root")
        self._ip = rospy.get_param("~router_ip","192.168.1.1")
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)
        self._timeout_counter = 0
        self._watchdogs = dict()
        for i in self._valid_ids:
            self._watchdogs[i] = watchdog(i)
    
        rospy.Subscriber("ogc/identifiers/valid_ids", UInt8MultiArray, self.update_valid_ids_cb)

    def update_valid_ids_cb(self, msg):
        self._valid_ids = [i for i in msg.data]

        del self._new_msg_chk
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)

        for i in self._watchdogs.keys():
            i.kill_timers()

        # clear and recrete self._watchdogs
        del self._watchdogs
        self._watchdogs = {}

        for i in self._valid_ids:
            self._watchdogs[i] = watchdog(i)

    ###########################
    # Watchdog handlers
    ###########################
    
    def _switch_all(self, link):
        '''Command ALL clients to switch to the target link (if not already on that link)'''
        for i in self._valid_ids:
            rospy.logwarn(f"Switching all clients to {link}")
            if not (self._watchdogs[i].link_status() == link):
                self._watchdogs[i].switch(link)

    ###########################
    # Link Monitors
    ###########################
    
    def _is_valid_msg(self, msg):
        '''Takes in a string msg. Return true + sender's sysid if msg is valid'''
        try:
            msgtype, devicetype, sysid, uuid, timestamp, entries \
                = headers.split_headers(msg)
            if not self._new_msg_chk.is_new_msg(timestamp, sysid):
                return False, 0
            else:
                return True, sysid
        except (ValueError, IndexError, TypeError):
            False, 0
    
    def _monitor_common(self, sysid, link):
        '''Manage link status of the client with the specified sysid'''
        # If active link is downstream of current link, trigger link switch to current link
        # Otherwise, simply reset the watchdog of the current link
        if self._watchdogs[sysid].link_status() > link:
            self._watchdogs[sysid].switch(link)
        else:
            self._watchdogs[sysid].reset_watchdog(link)
    
    def monitor_tele(self, data):
        '''Monitor telegram link for incoming msgs'''
        # Add optiimization methods here
        valid, sender_sysid = self._is_valid_msg(data.data)
        if valid:
            self._monitor_common(sender_sysid, TELE)

    def monitor_sms(self, data):
        '''Monitor sms link for incoming msgs'''
        # Add optiimization methods here
        valid, sender_sysid = self._is_valid_msg(data.data)
        if valid:
            self._monitor_common(sender_sysid, SMS)

    def monitor_router(self, data):
        ssh = RuTOS.start_client(self._ip, self._username)
        connection = RuTOS.get_conntype(ssh)
        rssi = RuTOS.get_rssi(ssh)
        if connection == "NOSERVICE":
            rospy.logerr("Router has no service. Switching to SBD")
            self._switch_all(SBD) # Switch to SBD immediately
        elif connection == "GSM":
            rospy.logerr("No data connection, switching to SMS")
            self._switch_all(SMS) # Switch to SMS
        elif rssi <= -85: # To include RSRQ, RSRP, SINR in the future
            for i in self._valid_ids:
                rospy.logerr("RSSI below threshold, switching link")
                old_link = self._watchdogs[i].link_status()
                self._watchdogs[i].switch(old_link + 1) # In 4G mode, switch at low rssi
    
    def monitor_teleout(self, data):
        if data.data == "Timeout":
            self._timeout_counter += 1
        elif data.data == "Success":
            self._timeout_counter = 0
            
        if self._timeout_counter == 3:
            self._timeout_counter = 0
            rospy.logerr("Telegram is taking too long to send. Switching to SMS")
            self._switch_all(SMS)

    def monitor_smsout(self, data):
        if data.data == "Timeout":
            rospy.logerr("SMS timed out. Link is no longer useable")
            SMS_timedout = True
            self._switch_all(SBD)

    ###########################
    # "Main" function
    ###########################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.monitor_sms)
        rospy.Subscriber("ogc/from_telegram", String, self.monitor_tele)
        rospy.Subscriber('ogc/to_switcher_tele', String, self.monitor_teleout)
        rospy.Subscriber('ogc/to_switcher_sms', String, self.monitor_smsout)
        # router_monitor = rospy.Timer(rospy.Duration(5), self.monitor_router)
        rospy.spin()
        # router_monitor.shutdown()

if __name__=='__main__':
    run = switcher()
    run.client()

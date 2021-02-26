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
from std_msgs.msg import String, UInt8MultiArray

from dynamic_delay import dynamic_delay
import headers
from identifiers.srv import GetIds
import RuTOS

TELE = 0
SMS = 1
SBD = 2

SIM = True # High latency simulator toggle
SMS_timedout = False #@TODO: Why is SMS_timedout declared here and then again in the switcher class???

if SIM:
    import csv
    import numpy

###########################
# Watchdog Class
###########################

class watchdog():
    '''An instance of watchdog should be created for each client'''
    def __init__(self, client_id):
        self.pub_to_despatcher = rospy.Publisher('ogc/from_switcher', String, queue_size=5)
        self._client_id = client_id
        self._link = TELE
        self._delay_calc = [None, None] # Dynamic delay calculator. One for tele, one for sms
        self._watchdog = [None, None] # Watchdog timer. One for tele, one for sms. When timer expires, link switch will trigger
        self._init_delay_calc_and_timer()
        self.countdown_handler = rospy.Timer(rospy.Duration(1), self.countdown)

    def _print_rto_update(self, link):
        rospy.loginfo(f"Switcher: Client {self._client_id}, link {link} watchdog set to {self._watchdog[link]} secs")
    
    def _init_delay_calc_and_timer(self):
        '''Initialise dynamic delay calculators and watchdog timers for each link in the watchdog'''
        link = 0
        while link <= SMS:
            self._delay_calc[link] = dynamic_delay()
            # Modify your respective link intervals here. @TODO: Break out all link interval handling into one area
            if link == TELE:
                self._delay_calc[link].set_interval(rospy.get_param("~interval_1"))
                self._delay_calc[link].set_recovery_interval_and_rto(rospy.get_param("~interval_2"))
            elif link == SMS:
                self._delay_calc[link].set_interval(rospy.get_param("~interval_2"))
                self._delay_calc[link].set_recovery_interval_and_rto(rospy.get_param("~interval_3"))
            self._delay_calc[link].reset_rto()
            self._watchdog[link] = self._delay_calc[link].get_rto()
            self._print_rto_update(link)
            link += 1

    def reset_watchdog(self, link, sent_timestamp):
        '''Calculate the new rto time and then reset watchdog to that time'''
        self._delay_calc[link].calc_rto(sent_timestamp)
        self._watchdog[link] = self._delay_calc[link].get_rto()
        self._print_rto_update(link)
    
    def switch(self, target_link):
        '''Perform the link-switch action and notify despatcher'''
        if target_link == SMS and SMS_timedout:
            target_link = SBD
        self._link = target_link
        rospy.logwarn(f"Switcher: Client {self._client_id} switching to link {target_link}")
        self.pub_to_despatcher.publish(str(self._client_id) + " " + str(self._link))
        if target_link <= SMS:
            self._delay_calc[target_link].reset_rto() # Set rto to its beginning value
            self._watchdog[target_link] = self._delay_calc[target_link].get_rto()
            self._print_rto_update(target_link)
    
    def countdown(self, data):
        '''Decrement the watchdog by 1 second. When watchdog expires, trigger the link switch'''
        if self._link >= SBD:
            return
        self._watchdog[self._link] = self._watchdog[self._link] - 1
        if self._watchdog[self._link] <= 0:
            rospy.logwarn(f"Switcher: Client {self._client_id}, link {self._link} watchdog expired")
            self._delay_calc[self._link].set_link_state(-1) # Tell delay calculator that this link is down
            self.switch(self._link + 1)
    
    def link_status(self):
        return self._link

    def kill_timers(self):
        self.countdown_handler.shutdown()

    def get_rto(self, link):
        return self._watchdog[link]

    def get_interval(self, link):
        return self._delay_calc[link].get_interval()

###########################
# Switcher Class
###########################

class switcher():

    def __init__(self):
        rospy.init_node('switcher', anonymous=False)
        self.pub_to_despatcher = rospy.Publisher('ogc/from_switcher', String, queue_size=5)
        rospy.wait_for_service("identifiers/get/valid_ids")
        ids_get_valid_ids = rospy.ServiceProxy("identifiers/get/valid_ids", GetIds)
        self._valid_ids = ids_get_valid_ids().ids
        self._username = rospy.get_param("~router_username","root")
        self._ip = rospy.get_param("~router_ip","192.168.1.1")
        self._ssh = RuTOS.start_client(self._ip, self._username)
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)
        self.SMS_timedout = False
        self._timeout_counter = 0
        self._watchdogs = dict()
        for i in self._valid_ids:
            self._watchdogs[i] = watchdog(i)
    
        rospy.Subscriber("ogc/identifiers/valid_ids", UInt8MultiArray, self.update_valid_ids_cb)
        
        # Log rto data if doing high latency simulation
        if SIM:
            env_home = '/home/lenovo' # Since this is only for simulation, we can hardcode the home directory
            self.csvfile = open(f'{env_home}/sim.csv', 'w')
            self.writer = csv.writer(self.csvfile)
            self.writer.writerow(['sent_timestamp', 'true latency', 'est latency'])
            self._fake_latency = 0

    def update_valid_ids_cb(self, msg):
        '''Callback function to obtain updated list of valid ids from the admin server'''
        self._valid_ids = [i for i in msg.data]
        del self._new_msg_chk
        self._new_msg_chk = headers.new_msg_chk(self._valid_ids)
        # clear and recrete self._watchdogs, since it is dependent on valid id list
        for i in self._watchdogs.values():
            i.kill_timers()
        del self._watchdogs
        self._watchdogs = {}
        for i in self._valid_ids:
            self._watchdogs[i] = watchdog(i)
    
    def _switch_all(self, link):
        '''Command ALL clients to switch to the target link (if not already on that link)'''
        for i in self._valid_ids:
            rospy.logwarn(f"Switcher: Switching all clients to {link}")
            if not (self._watchdogs[i].link_status() == link):
                self._watchdogs[i].switch(link)

    def _sim_latency(self):
        '''Generate simulated latency from a normal distribution'''
        mean = 1 # Edit mean and standard deviations here
        stdev = 0.1
        temp = numpy.random.normal(mean, stdev, 1)[0]
        if temp < 0:
            self._fake_latency = 0
        else:
            self._fake_latency = temp

    ###########################
    # Baseline Link Monitors
    ###########################
    
    def _is_valid_msg(self, msg):
        '''Takes in a string msg. Return true + sender's sysid + timestamp if msg is valid'''
        try:
            msgtype, devicetype, sysid, uuid, timestamp, entries \
                = headers.split_headers(msg)
            if not self._new_msg_chk.is_new_msg(timestamp, sysid):
                return False, 0, 0
            else:
                return True, sysid, timestamp
        except (ValueError, IndexError, TypeError):
            False, 0, 0
    
    def _monitor_common(self, sysid, link, sent_timestamp):
        '''Manage link status of the client with the specified sysid'''
        # If active link is downstream of current link, trigger link switch to current link and enter recovery phase
        # Otherwise, reset watchdog and update max_time of the current link
        if self._watchdogs[sysid].link_status() > link:
            self._watchdogs[sysid].switch(link)
        else:
            self._watchdogs[sysid].reset_watchdog(link, sent_timestamp)
        if SIM:
            rto = self._watchdogs[sysid].get_rto(link)
            inter = self._watchdogs[sysid].get_interval(link)
            rospy.loginfo(f"Switcher: Est latency = {rto}")
            rospy.loginfo(f"Switcher: True latency = {inter + self._fake_latency}")
            self.writer.writerow([sent_timestamp, inter + self._fake_latency, rto])

    def monitor_tele(self, data):
        '''Monitor telegram link for incoming msgs'''
        valid, sender_sysid, sent_timestamp = self._is_valid_msg(data.data)
        if valid:
            if SIM:
                self._sim_latency()
                sent_timestamp -= self._fake_latency # Simulate high latency
            self._monitor_common(sender_sysid, TELE, sent_timestamp)

    def monitor_sms(self, data):
        '''Monitor sms link for incoming msgs'''
        valid, sender_sysid, sent_timestamp = self._is_valid_msg(data.data)
        if valid:
            self._monitor_common(sender_sysid, SMS, sent_timestamp)

    ###########################
    # Optimisation Monitors
    ###########################
    
    def monitor_router(self, data):
        '''Check router for updates on connection status'''
        if self.SMS_timedout:
            return
        connection = RuTOS.get_conntype(self._ssh)
        rssi = RuTOS.get_rssi(self._ssh)
        if connection == "NOSERVICE":
            rospy.logerr("Switcher: No service, switching to SBD")
            self._switch_all(SBD) # Switch to SBD immediately
        elif connection == "GSM":
            rospy.logerr("Switcher: No data connection, switching to SMS")
            self._switch_all(SMS) # Switch to SMS
        elif rssi == "Timeout." or rssi == None:
            return
        elif rssi <= -95: # To include RSRQ, RSRP, SINR in the future
            for i in self._valid_ids:
                rospy.logerr("Switcher: Low RSSI, switching link")
                old_link = self._watchdogs[i].link_status()
                self._watchdogs[i].switch(old_link + 1) # In 4G mode, switch at low rssi
    
    def monitor_teleout(self, data):
        '''Obtain send status of outgoing tele msgs'''
        if data.data == "Timeout":
            self._timeout_counter += 1
        elif data.data == "Success":
            self._timeout_counter = 0
            
        if self._timeout_counter == 5:
            self._timeout_counter = 0
            rospy.logerr("Switcher: Telegram send timeout")
            for i in self._valid_ids:
                if self._watchdogs[i].link_status() == TELE:
                    self._watchdogs[i].switch(SMS)

    def monitor_smsout(self, data):
        '''Obtain send status of outgoing sms msgs'''
        if data.data == "Timeout":
            rospy.logerr("Switcher: SMS timeout. Link will restart in 8 minutes")
            self.SMS_timedout = True
            self._switch_all(SBD)
        if data.data == "Online":
            self.SMS_timedout = False

    ###########################
    # "Main" function
    ###########################
    
    def client(self):
        rospy.Subscriber("ogc/from_sms", String, self.monitor_sms)
        rospy.Subscriber("ogc/from_telegram", String, self.monitor_tele)
        rospy.Subscriber('ogc/to_switcher_tele', String, self.monitor_teleout)
        rospy.Subscriber('ogc/to_switcher_sms', String, self.monitor_smsout)
        router_monitor = rospy.Timer(rospy.Duration(5), self.monitor_router)
        rospy.spin()
        router_monitor.shutdown()
        self.csvfile.close()

if __name__=='__main__':
    run = switcher()
    run.client()

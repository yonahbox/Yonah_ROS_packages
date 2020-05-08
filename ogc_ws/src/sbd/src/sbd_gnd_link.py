#!/usr/bin/env python3

"""
sbd_link: ROS node to handle Iridium Short-Burst-Data (SBD) telemetry link on ground side.

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
import ast
import binascii
import datetime
import requests

# ROS/Third-Party
import rospy
from std_msgs.msg import String

class satcommsgnd():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        rospy.init_node('sbd_gnd_link', anonymous=False)
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)
        self.interval = rospy.get_param("~interval", "0.5") # sleep interval between checks for incoming msg
        self.buffer = self.buffer_write_time = "" # MO msg buffer and buffer write time
        self.server_url = "" # Our web server url
        self.prev_time = datetime.datetime.utcnow() # Time of previous msg received. Rock 7 uses UTC time
        self.mt_cred = dict() # Credentials required to post data to Rock 7 server
        self.cred_file = rospy.get_param("~credentials") # Text file containing credentials
        self.get_credentials()

    def get_credentials(self):
        '''Obtain Rock 7's required credentials from login.txt file'''
        # IMEI will be replaced with a list of IMEIs when we scale up to multiple aircraft
        with open(self.cred_file, 'r') as fp:
            self.server_url = fp.readline().replace('\n','') # Our web server url
            self.mt_cred['imei'] = fp.readline().replace('\n','') # Client Rockblock IMEI
            self.mt_cred['username'] = fp.readline().replace('\n','') # Our Rock 7 username
            self.mt_cred['password'] = fp.readline().replace('\n','') # Our Rock 7 pw

    ############################
    # Message handlers
    ############################
    
    def is_new_msg(self, transmit_time):
        '''See if MO msg from server is a new message'''
        # Convert string to datetime
        new_time = datetime.datetime.strptime(transmit_time, "%y-%m-%d %H:%M:%S")
        # Msg is new if its transmit time is later than last recorded transmit time
        if new_time > self.prev_time:
            self.prev_time = new_time
            return True
        return False
    
    def send_msg(self, data):
        '''Post MT msg to the Rock 7 server'''
        url = 'https://core.rock7.com/rockblock/MT'
        # Message to Rock 7 needs to be hex encoded
        encoded_msg = data.data.encode()
        self.mt_cred['data'] = binascii.hexlify(encoded_msg).decode()
        reply = requests.post(url, data=self.mt_cred)
        self.pub_to_despatcher.publish(reply.text)

    def recv_msg(self, data):
        '''Extract MO msg from our web server'''
        values = {'pw':'testpw'} # We cannot use our account pw, because http is unencrypted...
        # Send HTTP post request to Rock 7 server, incoming msg (if any) stored in reply
        reply_str = requests.post(self.server_url, data=values).text
        try:
            reply = ast.literal_eval(reply_str) # Convert string to dict
            if reply['imei'] == self.mt_cred['imei']: # ensure imei is valid
                if self.is_new_msg(reply['transmit_time']):
                    self.pub_to_despatcher.publish(reply['data'])
            else:
                rospy.logwarn("Received unknown msg from " + str(reply['imei']))
        except (ValueError):
            rospy.logwarn("Invalid message received")

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/to_sbd", String, self.send_msg)
        message_handler = rospy.Timer(rospy.Duration(self.interval), self.recv_msg)
        rospy.spin()
        message_handler.shutdown()

if __name__=='__main__':
    run = satcommsgnd()
    run.client()
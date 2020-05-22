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
import struct

# ROS/Third-Party
import rospy
from std_msgs.msg import String

# Local
from sbd_air_link import satcomms

class satcommsgnd(satcomms):

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        rospy.init_node('sbd_gnd_link', anonymous=False)
        self._is_air = False # True = Node runs on aircraft, False = Node runs on GCS
        self.__server_url = "" # Our web server url
        self.__prev_time = datetime.datetime.utcnow() # Time of previous msg received. Rock 7 uses UTC time
        self.__mt_cred = dict() # Credentials required to post MT data to Rock 7 server
        self.__cred_file = rospy.get_param("~credentials") # Text file containing credentials

        self.__get_credentials()
        self._init_variables()
        self.__serial_1 = (self._own_serial >> 16) & 0xFF
        self.__serial_2 = (self._own_serial >> 8) & 0xFF
        self.__serial_3 = self._own_serial & 0xFF

    def __get_credentials(self):
        '''Obtain Rock 7's required credentials from login.txt file'''
        # IMEI will be replaced with a list of IMEIs when we scale up to multiple aircraft
        with open(self.__cred_file, 'r') as fp:
            self.__server_url = fp.readline().replace('\n','') # Our web server url
            self.__mt_cred['imei'] = fp.readline().replace('\n','') # Client Rockblock IMEI
            self.__mt_cred['username'] = fp.readline().replace('\n','') # Our Rock 7 username
            self.__mt_cred['password'] = fp.readline().replace('\n','') # Our Rock 7 pw
    
    ############################
    # Server Message handlers.
    ############################
    
    def __server_is_new_msg(self, transmit_time):
        '''See if MO msg from server is a new message'''
        # Convert string to datetime
        new_time = datetime.datetime.strptime(transmit_time, "%y-%m-%d %H:%M:%S")
        # Msg is new if its transmit time is later than last recorded transmit time
        if new_time > self.__prev_time:
            self.__prev_time = new_time
            return True
        return False
    
    def __server_decode_mo_msg(self, msg):
        '''Decode the hex-encoded msg from the server'''
        response = binascii.unhexlify(msg)
        if msg[0] == ord('R'):
            # Unpack if it is binary-compressed regular payload
            if msg[1] == ord('B') and msg[2] == self.__serial_1\
                and msg[3] == self.__serial_2 and msg[4] == self.__serial_3:
                msg = msg[5:] # Strip Rockblock 2 Rockblock prefix if present
            struct_cmd = "> s H H H H H H H H H H H" # To-do: Break this into regular payload module
            return str(struct.unpack(struct_cmd, response))
        else:
            return response
    
    def __server_send_msg(self, data):
        '''Post MT msg to the Rock 7 server'''
        url = 'https://core.rock7.com/rockblock/MT'
        # Message to Rock 7 needs to be hex encoded
        encoded_msg = data.data.encode()
        self.__mt_cred['data'] = binascii.hexlify(encoded_msg).decode()
        reply = requests.post(url, data=self.__mt_cred)
        self._pub_to_despatcher.publish(reply.text)

    def __server_recv_msg(self):
        '''Extract MO msg from our web server'''
        values = {'pw':'testpw'} # We cannot use our account pw, because http is unencrypted...
        # Send HTTP post request to Rock 7 server, incoming msg (if any) stored in reply
        reply_str = requests.post(self.__server_url, data=values).text
        try:
            reply = ast.literal_eval(reply_str) # Convert string to dict
            if reply['imei'] == self.__mt_cred['imei']: # ensure imei is valid
                if self.__server_is_new_msg(reply['transmit_time']):
                    self._pub_to_despatcher.publish(self.__server_decode_mo_msg(reply['data']))
            else:
                rospy.logwarn("Received unknown msg from " + str(reply['imei']))
        except (ValueError):
            rospy.logwarn("Invalid message received")
    
    ############################
    # Main msg handlers
    ############################

    def send_msg(self, data):
        '''Handle MO msgs'''
        if self._thr_server:
            self.__server_send_msg(data)
        else:
            self._sbd_get_mo_msg(data)
    
    def recv_msg(self, data):
        '''Handle MT msgs'''
        if self._thr_server:
            self.__server_recv_msg()
        else:
            self._sbd_check_mailbox("") # "" is placeholder for data variable

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/to_sbd", String, self.send_msg)
        message_handler = rospy.Timer(rospy.Duration(self._interval), self.recv_msg)
        rospy.spin()
        message_handler.shutdown()

if __name__=='__main__':
    run = satcommsgnd()
    run.client()
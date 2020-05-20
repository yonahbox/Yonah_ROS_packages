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
import rockBlock
from rockBlock import rockBlockProtocol

class satcommsgnd():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        rospy.init_node('sbd_gnd_link', anonymous=False)
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)
        self.interval = rospy.get_param("~interval", "0.5") # sleep interval between checks for incoming msg
        self.buffer = self.buffer_write_time = "" # MO msg buffer and buffer write time
        self.thr_server = True # True = Comms through server; False = Comms through Rockblock (SBD)

        # Rockblock comms
        self.client_serial = rospy.get_param("~client_serial", "12345") # Rockblock serial no of client
        self.portID = rospy.get_param("~portID", "/dev/ttyUSB0") # Serial Port that Rockblock is connected to
        self.sbdsession = rockBlock.rockBlock(self.portID, self, self.client_serial) # Connect to Rockblock
        self.count = 0 # Mailbox check counter
        
        # Server comms
        self.server_url = "" # Our web server url
        self.prev_time = datetime.datetime.utcnow() # Time of previous msg received. Rock 7 uses UTC time
        self.mt_cred = dict() # Credentials required to post data to Rock 7 server
        self.cred_file = rospy.get_param("~credentials") # Text file containing credentials
        self._get_credentials()

    def _get_credentials(self):
        '''Obtain Rock 7's required credentials from login.txt file'''
        # IMEI will be replaced with a list of IMEIs when we scale up to multiple aircraft
        with open(self.cred_file, 'r') as fp:
            self.server_url = fp.readline().replace('\n','') # Our web server url
            self.mt_cred['imei'] = fp.readline().replace('\n','') # Client Rockblock IMEI
            self.mt_cred['username'] = fp.readline().replace('\n','') # Our Rock 7 username
            self.mt_cred['password'] = fp.readline().replace('\n','') # Our Rock 7 pw

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
        self.pub_to_despatcher.publish("Mailbox check failed, signal strength low")
    
    #MT
    def rockBlockRxStarted(self):
        self.pub_to_despatcher.publish("Starting mailbox check attempt " + str(self.count))

    def rockBlockRxFailed(self, mo_msg):
        if mo_msg == " ":
            self.pub_to_despatcher.publish("Mailbox check " + str(self.count) + " failed")
        else:
            self.pub_to_despatcher.publish("Mailbox check " + str(self.count) + " failed, message \'" +\
                mo_msg + "\' not delivered")

    def rockBlockRxReceived(self,mtmsn,data):
        self.pub_to_despatcher.publish(data)

    def rockBlockRxMessageQueue(self,count):
        rospy.loginfo("Rockblock found " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("Rockblock ready to send msg")

    def rockBlockTxFailed(self, momsg):
        rospy.logwarn("Rockblock msg not sent: " + momsg)

    def rockBlockTxSuccess(self,momsn, momsg):
        self.pub_to_despatcher.publish("SBD message sent: " + momsg)

    def rockBlockTxBlankMsg(self):
        self.pub_to_despatcher.publish("Mailbox check " + str(self.count) + " complete")
    
    ############################
    # Server Message handlers. Used when communicating with Iridium through our web server
    ############################
    
    def _server_is_new_msg(self, transmit_time):
        '''See if MO msg from server is a new message'''
        # Convert string to datetime
        new_time = datetime.datetime.strptime(transmit_time, "%y-%m-%d %H:%M:%S")
        # Msg is new if its transmit time is later than last recorded transmit time
        if new_time > self.prev_time:
            self.prev_time = new_time
            return True
        return False
    
    def _server_decode_mo_msg(self, msg):
        '''Decode the hex-encoded msg from the server'''
        response = binascii.unhexlify(msg)
        # Decode the binary-compressed regular payload. To-do: Add in ability to turn this on/off
        struct_cmd = "> H H H H H H H H H H H" # Compressed format of regular payload
        return str(struct.unpack(struct_cmd, response))
    
    def _server_send_msg(self, data):
        '''Post MT msg to the Rock 7 server'''
        url = 'https://core.rock7.com/rockblock/MT'
        # Message to Rock 7 needs to be hex encoded
        encoded_msg = data.data.encode()
        self.mt_cred['data'] = binascii.hexlify(encoded_msg).decode()
        reply = requests.post(url, data=self.mt_cred)
        self.pub_to_despatcher.publish(reply.text)

    def _server_recv_msg(self):
        '''Extract MO msg from our web server'''
        values = {'pw':'testpw'} # We cannot use our account pw, because http is unencrypted...
        # Send HTTP post request to Rock 7 server, incoming msg (if any) stored in reply
        reply_str = requests.post(self.server_url, data=values).text
        try:
            reply = ast.literal_eval(reply_str) # Convert string to dict
            if reply['imei'] == self.mt_cred['imei']: # ensure imei is valid
                if self._server_is_new_msg(reply['transmit_time']):
                    self.pub_to_despatcher.publish(self._server_decode_mo_msg(reply['data']))
            else:
                rospy.logwarn("Received unknown msg from " + str(reply['imei']))
        except (ValueError):
            rospy.logwarn("Invalid message received")

    ############################
    # Rockblock msg handlers. Used when communicating through ground Rockblock
    ############################
    
    def _sbd_get_mo_msg(self, data):
        '''
        Get MO msg from to_sbd topic and put it in MO buffer
        Note that MO msg will only be sent on next loop of check_sbd_mailbox
        '''
        self.buffer = data.data
        self.buffer_write_time = datetime.datetime.now()
    
    def _sbd_check_mailbox(self):
        '''Initiate an SBD mailbox check'''
        self.count = self.count + 1
        mailchk_time = datetime.datetime.now()
        # If no MO msg, buffer will be empty
        self.sbdsession.messageCheck(self.buffer)
        # Clear buffer if no new MO msgs were received after sending the previous MO msg
        if self.buffer_write_time < mailchk_time:
            self.buffer = ""
    
    ############################
    # Main msg handlers
    ############################

    def send_msg(self, data):
        '''Handle MO msgs'''
        if self.thr_server:
            self._server_send_msg(data)
        else:
            self._sbd_get_mo_msg(data)
    
    def recv_msg(self, data):
        '''Handle MT msgs'''
        if self.thr_server:
            self._server_recv_msg()
        else:
            self._sbd_check_mailbox()

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
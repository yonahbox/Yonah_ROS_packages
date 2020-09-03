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
import paramiko
import requests
import struct

# ROS/Third-Party
import rospy
from std_msgs.msg import String
from despatcher.msg import LinkMessage
from identifiers.srv import GetDetails, CheckSender, GetSBDDetails, GetIds

# Local
from sbd_air_link import satcomms
from regular import struct_cmd, convert_to_str

class satcommsgnd(satcomms):

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        rospy.init_node('sbd_gnd_link', anonymous=False)

        rospy.wait_for_service("identifiers/get/ids")
        rospy.wait_for_service("identifiers/get/imei")
        rospy.wait_for_service("identifiers/self/sbd")
        rospy.wait_for_service("identifiers/check/proper")

        self._get_ids = rospy.ServiceProxy("identifiers/get/ids", GetIds)
        self._get_imei = rospy.ServiceProxy("identifiers/get/imei", GetDetails)
        self._is_valid_sender = rospy.ServiceProxy("identifiers/check/proper", CheckSender)
        get_sbd_credentials = rospy.ServiceProxy("identifiers/self/sbd", GetSBDDetails)

        self._init_variables()

        # Switch state: Temporary state where gnd node is switching between server and ground rockblock
        self._switch_state = False # True = switch state is active
        self._id = rospy.get_param("~self_id") # Our GCS ID
        self._is_air = 0 # We are a ground node!

        # Three least significant bytes of own serial, used for binary unpack of regular payload
        self._serial_0 = (self._own_serial >> 16) & 0xFF
        self._serial_1 = (self._own_serial >> 8) & 0xFF
        self._serial_2 = self._own_serial & 0xFF

        # Comms with web server
        self._prev_time = datetime.datetime.utcnow() # Time of previous msg received. Rock 7 uses UTC time
        self._mt_cred = { # Credentials required to post MT data to Rock 7 server
            'imei': 0,
            'username': 'qwer',
            'password': 'qwert',
            'data': 'qwerty'
        }
        sbd_details = get_sbd_credentials()
        self._mt_cred['username'] = sbd_details.username
        self._mt_cred['password'] = sbd_details.password
        self._svr_ip = sbd_details.svr_ip
        self._svr_hostname = sbd_details.svr_hostname
        self._svr_buffer = "/satcomms_server/buffer.txt" # File in web server that stores MO msg
        if not self._ssh_conn(self._svr_ip, self._svr_hostname):
            self._thr_server = 0
            self._switch_cmd_handler()
    
    def _ssh_conn(self, ip, hostname):
        '''Establish ssh connection to web server. Return True if connection established'''
        self._ssh = paramiko.SSHClient()
        self._ssh.load_system_host_keys()
        try:
            self._ssh.connect(ip, username=hostname, timeout=10)
            rospy.loginfo("SBD: Connected to AWS Server")
            return True
        except:
            rospy.logerr("SBD: Cannot connect to AWS Server")
            return False
    
    ############################
    # Server Message handlers.
    ############################
    
    def _server_is_new_msg(self, transmit_time):
        '''See if MO msg from server is a new message'''
        new_time = datetime.datetime.strptime(transmit_time, "%y-%m-%d %H:%M:%S")
        # Msg is new if its transmit time is later than last recorded transmit time
        if new_time > self._prev_time:
            self._prev_time = new_time
            return True
        return False
    
    def _server_decode_mo_msg(self, msg):
        '''Decode the hex-encoded msg from the server'''
        response = binascii.unhexlify(msg)
        if len(response) >= 5 and response[0] == ord('R')and response[1] == ord('B') and \
        response[2] == self._serial_0 and response[3] == self._serial_1 and response[4] == self._serial_2:
            # Sometimes, msgs sent to another Rockblock also end up in the web server
            # Hence, strip Rockblock 2 Rockblock prefix if present
            response = response[5:]
        if response[0] == ord('r'):
            # If msg is binary compressed regular payload, unpack it accordingly
            return convert_to_str(struct.unpack(struct_cmd, response))
        else:
            # Everything else is non-regular-payload in ASCII form
            if len(response) > 9 and response.startswith(b'RB00'):
                # Sometimes, msgs sent to another Rockblock also end up in the web server
                # Hence, strip Rockblock 2 Rockblock prefix if present
                response = response[9:]
            # @TODO: Catch decode exceptions (e.g. accidental or malicious errors)
            return response.decode()
    
    def _server_send_msg(self, data):
        '''Post outgoing msg to the Rock 7 server'''
        url = 'https://core.rock7.com/rockblock/MT'
        # Message to Rock 7 needs to be hex encoded
        encoded_msg = data.data.encode()
        self._mt_cred['data'] = binascii.hexlify(encoded_msg).decode()
        imei = self._get_imei(data.id)
        if not imei.data:
            rospy.logwarn("Invalid recipient")
            return True
        self._mt_cred['imei'] = imei.data
        try:
            reply = requests.post(url, data=self._mt_cred)
            rospy.loginfo(reply.text)
            return True
        except:
            rospy.logerr("SBD: Cannot contact Rock 7 server")
            return False

    def _server_recv_msg(self):
        '''Extract incoming msg from our web server. Return True if able to connect to server'''
        try:
            # Extract msg from server's MO buffer file
            _, stdout, _ = self._ssh.exec_command("cat /satcomms_server/buffer.txt", timeout = 5)
            reply_str = str(stdout.readlines())[2:-4] # Remove padding
        except:
            rospy.logerr("SBD: Connection to AWS server lost")
            return False
        try:
            reply = ast.literal_eval(reply_str) # Convert string to dict
            rospy.loginfo(reply)
            if self._server_is_new_msg(reply['transmit_time']): 
                if self._is_valid_sender(link=2, details=reply['serial']): # ensure rb serial is valid
                    self._pub_to_despatcher.publish(self._server_decode_mo_msg(reply['data']))
                else:
                    rospy.logwarn("Received unknown msg from Rockblock " + str(reply['serial']))
        except (SyntaxError):
            rospy.logwarn("Web Server Syntax Error")
        except (ValueError):
            rospy.logwarn("Web Server Message Value Error")
        return True
    
    ############################
    # Handle switching between server and ground rockblock comms on aircraft
    ############################

    def _switch_cmd_handler(self):
        '''Send cmds to all aircraft, telling them to switch between server and RB-2-RB methods'''
        air_ids = self._get_ids().air_ids
        switch_cmd = LinkMessage()
        for i in air_ids:
            switch_cmd.id = i
            rospy.loginfo("Sending to aircraft " + str(i))
            if self._thr_server:
                switch_cmd.data = "e 0 " + str(self._id) + " sbd switch 1 " + str(rospy.get_rostime().secs)
                self._server_send_msg(switch_cmd)
            else:
                # Sending from gnd Rockblock is likely to fail. We need to ensure that all switch cmds are sent
                while not self._msg_send_success == 1:
                    switch_cmd.data = "e 0 " + str(self._id) + " sbd switch 0 " + str(rospy.get_rostime().secs)
                    self.sbd_get_mo_msg(switch_cmd)
                    self.sbd_check_mailbox("")

    def change_switch_state(self, data):
        '''Handle switching between server and RB-2-RB methods'''
        if self._switch_state:
            # If switch state activated, temporarily shutdown message handler to send switch cmds quickly
            rospy.loginfo("SBD: Entering switch state")
            self.message_handler.shutdown()
            self._switch_cmd_handler()
            self.message_handler = rospy.Timer(rospy.Duration(self.interval), self.recv_msg)
            self._switch_state = False
            rospy.loginfo("SBD: Leaving switch state")
    
    ############################
    # Main msg handlers
    ############################

    def send_msg(self, data):
        '''Handle outgoing msgs'''
        # Try sending through Rock 7 server first. If it fails, fallback to gnd rockBlock
        if not self._server_send_msg(data):
            self.sbd_get_mo_msg(data)
    
    def recv_msg(self, data):
        '''Handle incoming msgs'''
        if self._thr_server:
            if not self._ssh:
                self._ssh_conn(self._svr_ip, self._svr_hostname)
            # If can't contact server, switch to ground rockBlock
            if not self._server_recv_msg():
                self._switch_state = True
                self._thr_server = 0
        else:
            # Make way for switch state if it is active
            if not self._switch_state:
                self.sbd_check_mailbox("") # "" is placeholder for data variable
                # If can contact server, switch to server method for next iteration
                if self._ssh_conn(self._svr_ip, self._svr_hostname):
                    self._switch_state = True
                    self._thr_server = 1

    ############################
    # "Main" function
    ############################
    
    def client(self):
        rospy.Subscriber("ogc/to_sbd", LinkMessage, self.send_msg)
        self.message_handler = rospy.Timer(rospy.Duration(self.interval), self.recv_msg)
        switch_state_handler = rospy.Timer(rospy.Duration(0.5), self.change_switch_state)
        rospy.spin()
        self.message_handler.shutdown()
        switch_state_handler.shutdown()
        self._ssh.close()
        self._sbdsession.close()

if __name__=='__main__':
    run = satcommsgnd()
    run.client()
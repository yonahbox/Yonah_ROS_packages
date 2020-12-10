#!/usr/bin/env python3
"""
sms_link: ROS node to handle SMS telemetry link on both air and ground side.

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
import paramiko

# ROS/Third-Party
import rospy
from std_msgs.msg import String
from despatcher.msg import LinkMessage
from identifiers import Identifiers
from identifiers.srv import CheckSender, GetDetails

# Local
import RuTOS
import sys
import timeoutscript


class SMSrx():
    
    ############################
    # Initialization
    ############################

    def __init__(self):
        rospy.init_node('sms_link', anonymous=False)
        self._username = rospy.get_param("~router_username","root") # Hostname of onboard router
        self._ip = rospy.get_param("~router_ip","192.168.1.1") # IP Adress of onboard router
        self._msglist = "" # Raw incoming message extracted by router (see https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index)
        self._msg = "" # Actual incoming message, located on 5th line of msglist
        self.interval = 1 # Time interval between each check of the router for incoming msgs

        # Initialise SSH
        try:
            self.ssh = RuTOS.start_client(self._ip, self._username)
            rospy.loginfo("Connected to router")
        except:
            rospy.logerr("Could not connect to the router")
            raise

        # Initialize publisher to despatcher nodes
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sms', String, queue_size = 5)
        
        # Publish to switcher
        self.pub_to_switcher = rospy.Publisher('ogc/to_switcher_sms', String, queue_size=5)

        # Publish to timeout module
        self.pub_to_timeout = rospy.Publisher('ogc/to_timeout', LinkMessage, queue_size = 5)

        # identifiers work
        rospy.wait_for_service("identifiers/check/proper")
        rospy.wait_for_service("identifiers/get/number")
        self._identifiers_valid_sender = rospy.ServiceProxy("identifiers/check/proper", CheckSender)
        self._identifiers_get_number = rospy.ServiceProxy("identifiers/get/number", GetDetails)
        
        # Security and safety measures
        self._purge_residual_sms()

    def _purge_residual_sms(self):
        """
        Clear the router of SMSes (capped at 30 for the RUT955) on bootup. This prevents the situation
        where an SMS is received when the air router was off, and is acted upon the moment the router
        switches on (very dangerous if that SMS is an arm msg!)
        """
        count = 1
        rospy.loginfo("Purging residual SMS, please wait...")
        while count <= 30:
            try:
                RuTOS.delete_msg(self.ssh, count)
                count += 1
            except:
                count += 1
                continue
        rospy.loginfo("Purge complete!")

    #########################################
    # Handle incoming/outgoing SMS messages
    #########################################
    
    def send_sms(self, data):
        '''
        Send msg from despatcher node (over ogc/to_sms topic) as an SMS
        '''
        rospy.loginfo("Sending SMS: " + data.data)
        number = self._identifiers_get_number(data.id)
        if number is None:
            rospy.logerr("Invalid ID number")
            return

        sendstatus = RuTOS.send_msg(self.ssh, "+"+number.data, data.data)
        
        # Send acknowledgment to timeout module
        ack = timeoutscript.ack_converter(data, 0)
        if ack != None:
            self.pub_to_timeout.publish(ack)

        if "Timeout\n" in sendstatus:
            self.pub_to_switcher.publish("Timeout")
            rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
        elif "Connection lost" in sendstatus:
            rospy.logerr("Connection to router lost!")
        else:
            ack = timeoutscript.ack_converter(data, 1)
            if ack != None:
                self.pub_to_timeout.publish(ack)
            self.pub_to_switcher.publish("Success")
    
    def recv_sms(self, data):
        '''Receive incoming SMS, process it, and forward to despatcher node via ogc/from_sms topic'''
        # Read an SMS received by the air router
        self._msglist = RuTOS.extract_msg(self.ssh, 1)
        if 'no message\n' in self._msglist:
            pass
        elif 'N/A\n' in self._msglist:
            pass
        elif 'Timeout.\n' in self._msglist:
            rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
        elif 'Connection lost' in self._msglist:
            rospy.logerr("Connection to router lost!")
        else:
            # extract sender number (2nd word of 3rd line in msglist)
            sender = self._msglist[2].split()[1]
            rospy.loginfo("Received sms from " + sender)
            # Ensure sender is whitelisted before extracting message
            is_valid_sender = self._identifiers_valid_sender(1, sender)
            if is_valid_sender.result:
            # if self._ids.is_valid_sender(1, sender):
                rospy.loginfo('Command from '+ sender)
                # msg is located on the 5th line (minus first word) of msglist. It is converted to lowercase
                self._msg = self._msglist[4].split(' ', 1)[1].rstrip()
                # Forward msg to air_despatcher
                self.pub_to_despatcher.publish(self._msg)
            else:
                rospy.logwarn('Rejected msg from unknown sender ' + sender)
            RuTOS.delete_msg(self.ssh, 1) # Delete the existing SMS
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        """Main function to let aircraft receive SMS commands"""
        rospy.Subscriber("ogc/to_sms", LinkMessage, self.send_sms)
        message_sender = rospy.Timer(rospy.Duration(self.interval), self.recv_sms)
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    try:
        run = SMSrx()
        run.client()
    except:
        rospy.loginfo("Failed to start node")
        raise
    else:
        run.ssh.close()
        rospy.loginfo("Connection to router closed")
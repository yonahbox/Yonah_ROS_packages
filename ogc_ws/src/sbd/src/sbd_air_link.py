#!/usr/bin/env python3

"""
sbd_link: ROS node to handle Iridium Short-Burst-Data (SBD) telemetry link on both air and ground side.

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

# ROS/Third-Party
import rospy
from std_msgs.msg import String
from despatcher.msg import LinkMessage

# Local
# from identifiers import Identifiers
from identifiers.srv import GetSelfDetails, GetDetails, CheckSender
import rockBlock
from rockBlock import rockBlockProtocol, rockBlockException

class local_mo_buffer():
    '''Buffer to hold MO msg locally before a mailbox check takes place'''
    def __init__(self):
        self.data = "" # actual msg
        self.msgtype = "r" # Msg type of the message that is already in the buffer
        self.write_time = rospy.get_rostime().secs # Time which msg is written to buffer
        self.target_id = 1 # ID of the client which we are sending to

class satcomms(rockBlockProtocol):

    def __init__(self):
        rospy.init_node('sbd_air_link', anonymous=False)
        
        rospy.wait_for_service("identifiers/self/serial")
        rospy.wait_for_service("identifiers/get/serial")
        rospy.wait_for_service("identifiers/check/lazy")

        self._init_variables()
        self._is_air = 1 # We are an air node!
        self._prev_switch_cmd_time = rospy.get_rostime().secs # Transmit time of previous incoming switch cmd
    
    def _init_variables(self):
        self._pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)

        # Identifiers
        self._get_self_serial = rospy.ServiceProxy("identifiers/self/serial", GetSelfDetails)
        self._get_serial = rospy.ServiceProxy("identifiers/get/serial", GetDetails)
        self._check_lazy = rospy.ServiceProxy("identifiers/check/lazy", CheckSender)

        # Rockblock Comms
        self._buffer = local_mo_buffer()
        self._own_serial = int(self._get_self_serial().data) # Our own rockblock serial
        self._thr_server = rospy.get_param("~thr_server", "1") # 1 = Comm through web server; 0 = Comm through gnd Rockblock
        self._portID = rospy.get_param("~portID", "/dev/ttyUSB0") # Serial Port that Rockblock is connected to
        self._count = 0 # Mailbox check counter
        self._msg_send_success = 0 # Check if MO msg successsfully sent. 0 = pending/N.A, -1 = unsuccessful, 1 = successful
        self.interval = rospy.get_param("~interval", "0.5") # sleep interval between mailbox checks
        try:
            self._sbdsession = rockBlock.rockBlock(self._portID, self._own_serial, self)
        except rockBlockException:
            rospy.signal_shutdown("rockBlock error")
        
        # Msg Type Prioritization. Higher number means higher priority
        self._msg_priority = {
            "r": 0,
            "a": 1,
            "m": 2,
            "s": 3,
            "i": 4,
            "w": 5,
            "e": 6
        }

    ################################
    # pyrockBlock callback functions
    ################################
    
    def rockBlockConnected(self):
        rospy.loginfo("Rockblock Connected")

    #SIGNAL
    def rockBlockSignalUpdate(self,signal):
        rospy.loginfo("Rockblock signal strength: " + str(signal))

    def rockBlockSignalPass(self):
        rospy.loginfo("Rockblock signal strength good")

    def rockBlockSignalFail(self):
        rospy.logwarn("Mailbox check failed, signal strength low")
    
    #MT
    def rockBlockRxStarted(self):
        rospy.loginfo("Starting mailbox check attempt " + str(self._count))

    def rockBlockRxFailed(self, mo_msg):
        if mo_msg == " ":
            rospy.logwarn("Mailbox check " + str(self._count) + " failed")
        else:
            rospy.logwarn("Mailbox check " + str(self._count) + " failed, message \'" +\
                mo_msg + "\' not delivered")

    def rockBlockRxReceived(self,mtmsn,data):
        check_result = self._check_lazy(details=data)
        if check_result.result:
            # Act on switch cmds immediately and publish the rest to despatcher
            if self._is_air:
                is_switch_cmd = self._check_switch_cmd(data)
            else:
                is_switch_cmd = False
            if not is_switch_cmd:
                self._pub_to_despatcher.publish(data)

    def rockBlockRxMessageQueue(self,count):
        rospy.loginfo("Rockblock found " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("Rockblock ready to send msg")

    def rockBlockTxFailed(self, momsg):
        rospy.logwarn("Rockblock msg not sent: " + momsg)
        self._msg_send_success = -1

    def rockBlockTxSuccess(self,momsn, momsg):
        rospy.loginfo("SBD message sent: " + momsg)
        self._msg_send_success = 1

    def rockBlockTxBlankMsg(self):
        rospy.loginfo("Mailbox check " + str(self._count) + " complete")

    ############################
    # Check for switch cmds
    ############################

    def _check_switch_cmd(self, data):
        '''Check if there is a need to switch between server and RB-2-RB comms. Return True if switch was made'''
        if "sbd switch 0" in data:
            switch_cmd_time = data.split()[-1]
            if switch_cmd_time > self._prev_switch_cmd_time:
                self._prev_switch_cmd_time = switch_cmd_time
                self._thr_server = 0
                return True
        if "sbd switch 1" in data:
            switch_cmd_time = data.split()[-1]
            if switch_cmd_time > self._prev_switch_cmd_time:
                self._prev_switch_cmd_time = switch_cmd_time
                self._thr_server = 1
                return True
        return False

    ############################
    # Rockblock MO/MT msg calls
    ############################
    
    def sbd_get_mo_msg(self, data):
        '''
        Get MO msg from to_sbd topic and put it in local MO buffer depending on its priority level
        Note that MO msg will only be sent on next loop of check_sbd_mailbox
        '''
        incoming_msgtype = data.data.split()[0]
        # Reject incoming msg if existing msg in the local buffer is already of a higher priority
        if (self._msg_priority[incoming_msgtype] < self._msg_priority[self._buffer.msgtype]):
            rospy.logdebug("Reject incoming msg " + data.data)
            rospy.loginfo("Existing msg in SBD local MO buffer is of higher priority: " + self._buffer.data)
            return
        self._buffer.data = data.data
        self._buffer.msgtype = incoming_msgtype
        self._buffer.write_time = rospy.get_rostime().secs
        self._buffer.target_id = data.id
    
    def sbd_check_mailbox(self, data):
        '''
        Initiate an SBD mailbox check.
        This checks for incoming Mobile-Terminated (MT) msgs,
        and sends outgoing Mobile-Originated (MO) msgs if there are any in the buffer
        '''
        # If no MO msg, local MO buffer will be empty
        self._count = self._count + 1
        self._msg_send_success = 0
        mailchk_time = rospy.get_rostime().secs
        # Get RB serial number of client
        client_serial = self._get_serial(self._buffer.target_id).data
        if not client_serial:
            rospy.logwarn("Invalid recipient. Proceeding with mailbox check without target receipient")
        # If buffer starts with "r ", it is a regular payload
        mo_is_regular = False
        if self._buffer.data.startswith("r "):
            mo_is_regular = True
        try:
            self._sbdsession.messageCheck(self._buffer.data, client_serial, self._thr_server, mo_is_regular)
        except rockBlockException:
            # Restart the node if error occurs
            rospy.signal_shutdown("rockBlock error")
        # Clear buffer and reset priority to lowest, if no new MO msgs were received after sending the previous one
        if self._buffer.write_time < mailchk_time:
            self._buffer.data = ""
            self._buffer.msgtype = "r"

    ############################
    # "Main" function
    ############################
    
    def air_client(self):
        rospy.Subscriber("ogc/to_sbd", LinkMessage, self.sbd_get_mo_msg)
        message_handler = rospy.Timer(rospy.Duration(self.interval), self.sbd_check_mailbox)
        rospy.spin()
        message_handler.shutdown()
        self._sbdsession.close()

if __name__=='__main__':
    run = satcomms()
    run.air_client()
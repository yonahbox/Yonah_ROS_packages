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
from identifiers.srv import GetSelfDetails, GetDetails, CheckSender, GetIds
import headers
import rockBlock
from rockBlock import rockBlockProtocol, rockBlockException

import timeoutscript
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
        
        rospy.wait_for_service("identifiers/get/valid_ids")

        ids_get_valid_ids = rospy.ServiceProxy("identifiers/get/valid_ids", GetIds)
        _valid_ids = ids_get_valid_ids().ids

        self._init_variables()
        self._is_air = 1 # We are an air node!
        
        # Put the publisher function here
        self.pub_to_timeout = rospy.Publisher('ogc/to_timeout', LinkMessage, queue_size = 5)
        # Headers (for checking of switch cmds)
        self._new_switch_cmd = headers.new_msg_chk(_valid_ids)
    
    def _init_variables(self):
        self._pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)

        # Identifiers
        rospy.wait_for_service("identifiers/self/serial")
        rospy.wait_for_service("identifiers/get/serial")
        rospy.wait_for_service("identifiers/check/lazy")

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
            "h": 0,
            "r": 1,
            "a": 2,
            "m": 3,
            "s": 4,
            "i": 5,
            "w": 6,
            "e": 7
        }

    ################################
    # pyrockBlock callback functions
    ################################
    
    def rockBlockConnected(self):
        rospy.loginfo("SBD: Rockblock Connected")

    #SIGNAL
    def rockBlockSignalUpdate(self,signal):
        rospy.loginfo("SBD: Iridium signal strength = " + str(signal))

    def rockBlockSignalPass(self):
        rospy.loginfo("SBD: Signal strength good")

    def rockBlockSignalFail(self):
        rospy.logwarn("SBD: Mailbox check failed, signal strength low")
    
    #MT
    def rockBlockRxStarted(self):
        rospy.loginfo("SBD: Starting mailbox check " + str(self._count))

    def rockBlockRxFailed(self, mo_msg):
        if mo_msg == " ":
            rospy.logwarn("SBD: Mailbox check " + str(self._count) + " failed")
        else:
            rospy.logwarn("SBD: Mailbox check " + str(self._count) + " failed, message \'" +\
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
        rospy.loginfo("SBD: " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("SBD: Rockblock Tx ready")

    def rockBlockTxFailed(self, momsg):
        rospy.logwarn("SBD: Msg not sent: " + momsg)
        self._msg_send_success = -1

    def rockBlockTxSuccess(self,momsn, momsg):
        ack = timeoutscript.ack_converter(msg, 1)
        if ack != None:
            self.pub_to_timeout.publish(ack)
        rospy.loginfo("SBD: Msg sent: " + momsg)
        self._msg_send_success = 1

    def rockBlockTxBlankMsg(self):
        rospy.loginfo("SBD: Mailbox check " + str(self._count) + " complete")

    ############################
    # Check for switch cmds
    ############################

    def _check_switch_cmd(self, data):
        '''Check if there is a need to switch between server and RB-2-RB comms. Return True if switch was made'''
        if "sbd switch 0" in data:
            _, _, sysid, _, switch_cmd_time, _ = headers.split_headers(data)
            if self._new_switch_cmd.is_new_msg(switch_cmd_time, sysid):
                self._prev_switch_cmd_time = switch_cmd_time
                self._thr_server = 0
                rospy.loginfo("SBD: Switching to RB-2-RB comms")
                return True
        if "sbd switch 1" in data:
            _, _, sysid, _, switch_cmd_time, _ = headers.split_headers(data)
            if self._new_switch_cmd.is_new_msg(switch_cmd_time, sysid):
                self._prev_switch_cmd_time = switch_cmd_time
                self._thr_server = 1
                rospy.loginfo("SBD: Switching to Server comms")
                return True
        return False

    ############################
    # Rockblock MO/MT msg calls
    ############################
    
    def sbd_get_mo_msg(self, data): # Add here
        '''
        Get MO msg from to_sbd topic and put it in local MO buffer depending on its priority level
        Note that MO msg will only be sent on next loop of check_sbd_mailbox
        '''
        incoming_msgtype,_,_,_,_,_ = headers.split_headers(data.data)
        # Reject incoming msg if existing msg in the local buffer is already of a higher priority
        if (self._msg_priority[incoming_msgtype] < self._msg_priority[self._buffer.msgtype]):
            rospy.loginfo("SBD: Reject incoming msg " + data.data)
            rospy.loginfo("SBD: Existing msg in SBD local MO buffer is of higher priority: " + self._buffer.data)
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
            rospy.logwarn("SBD: Invalid receipient serial. Proceeding with mailbox check without target receipient")
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
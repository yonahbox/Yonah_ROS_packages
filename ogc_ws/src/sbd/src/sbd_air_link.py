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
from identifiers import Identifiers
import rockBlock
from rockBlock import rockBlockProtocol, rockBlockException

class mo_msg_buffer():
    '''Buffer to hold MO msg before a mailbox check takes place'''
    def __init__(self):
        self.data = "" # actual msg
        self.write_time = rospy.get_rostime().secs # Time which msg is written to buffer
        self.target_id = -1 # ID of the client which we are sending to

class satcomms(rockBlockProtocol):

    def __init__(self):
        rospy.init_node('sbd_link', anonymous=False)
        self._init_variables()
    
    def _init_variables(self):
        self._pub_to_despatcher = rospy.Publisher('ogc/from_sbd', String, queue_size = 5)

        # Identifiers
        identifiers_file = rospy.get_param("~identifiers_file")
        self._valid_ids = rospy.get_param("~valid_ids")
        self._is_air = rospy.get_param("~is_air")
        self._self_id = rospy.get_param("~self_id")
        self._ids = Identifiers(identifiers_file, self._is_air, self._valid_ids)
        self._own_serial = self._ids.get_sbd_serial(self._self_id) # Our Rockblock IMEI & Serial

        # Rockblock Comms
        self._buffer = mo_msg_buffer()
        self._thr_server = False # True = Comm through web server; False = Comm through gnd Rockblock
        self._portID = rospy.get_param("~portID", "/dev/ttyUSB0") # Serial Port that Rockblock is connected to
        self._count = 0 # Mailbox check counter
        self.interval = rospy.get_param("~interval", "0.5") # sleep interval between mailbox checks
        try:
            self._sbdsession = rockBlock.rockBlock(self._portID, self, self._own_serial)
        except rockBlockException:
            rospy.signal_shutdown("rockBlock error")

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
        self._pub_to_despatcher.publish(data)

    def rockBlockRxMessageQueue(self,count):
        rospy.loginfo("Rockblock found " + str(count) + " queued incoming msgs")
     
    #MO
    def rockBlockTxStarted(self):
        rospy.loginfo("Rockblock ready to send msg")

    def rockBlockTxFailed(self, momsg):
        rospy.logwarn("Rockblock msg not sent: " + momsg)

    def rockBlockTxSuccess(self,momsn, momsg):
        rospy.loginfo("SBD message sent: " + momsg)

    def rockBlockTxBlankMsg(self):
        rospy.loginfo("Mailbox check " + str(self._count) + " complete")

    ############################
    # Rockblock MO/MT msg calls
    ############################
    
    def sbd_get_mo_msg(self, data):
        '''
        Get MO msg from to_sbd topic and put it in MO buffer
        Note that MO msg will only be sent on next loop of check_sbd_mailbox
        '''
        self._buffer.data = data.data
        self._buffer.write_time = rospy.get_rostime().secs
        self._buffer.target_id = data.id
    
    def sbd_check_mailbox(self, data):
        '''
        Initiate an SBD mailbox check.
        This checks for incoming Mobile-Terminated (MT) msgs,
        and sends outgoing Mobile-Originated (MO) msgs if there are any in the buffer
        '''
        # If no MO msg, buffer will be empty
        self._count = self._count + 1
        mailchk_time = rospy.get_rostime().secs
        # Get RB serial number of client
        client_serial = self._ids.get_sbd_serial(self._buffer.target_id)
        # print(client_serial)
        # if not client_serial:
        #     rospy.logwarn("Invalid recipient")
        #     return
        # If buffer starts with "r ", it is a regular payload
        mo_is_regular = False
        if self._buffer.data.startswith("r "):
            mo_is_regular = True
        try:
            self._sbdsession.messageCheck(self._buffer.data, client_serial, self._thr_server, mo_is_regular)
        except rockBlockException:
            # Restart the node if error occurs
            rospy.signal_shutdown("rockBlock error")
        # Clear buffer if no new MO msgs were received after sending the previous MO msg
        if self._buffer.write_time < mailchk_time:
            self._buffer.data = ""

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
#!/usr/bin/env python3
"""
sms_link

ROS node to handle SMS telemetry link on both air and ground side.

Lau Yan Han and Yonah, Apr 2020

"""

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from std_msgs.msg import String

# Local
import RuTOS

class SMSrx():
    
    ############################
    # Initialization
    ############################

    def __init__(self):
        rospy.init_node('sms_link', anonymous=False)
        self.router_hostname = rospy.get_param("~router_hostname","root@192.168.1.1") # Hostname and IP of onboard router
        self.whitelist = set() # set of whitelisted numbers
        self.client_no = rospy.get_param("~client_phone_no", "12345678") # GCS phone number
        self.msglist = "" # Raw incoming message extracted by router (see https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index)
        self.msg = "" # Actual incoming message, located on 5th line of msglist
        self.interval = 0.5 # Time interval between each check of the router for incoming msgs

        # Initialize publisher to despatcher nodes
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sms', String, queue_size = 5)
        
        # Security and safety measures
        self.populatewhitelist()
        #self.purge_residual_sms()

    def populatewhitelist(self):
        """Fill up whitelisted numbers. Note that whitelist.txt must be in same folder as this script"""
        textfile = rospy.get_param("~whitelist", "whitelist.txt")
        with open (textfile, "r") as reader:
            for line in reader:
                if line[-1] == "\n":
                    self.whitelist.add(line[:-1]) # remove whitespace at end of line
                else:
                    self.whitelist.add(line) # last line
        rospy.loginfo(self.whitelist)

    def purge_residual_sms(self):
        """
        Clear the router of SMSes (capped at 30 for the RUT955) on bootup. This prevents the situation
        where an SMS is received when the air router was off, and is acted upon the moment the router
        switches on (very dangerous if that SMS is an arm msg!)
        """
        # The bad thing about this implementation is that it increases boot time by 30 seconds
        count = 1
        rospy.loginfo("Purging residual SMS, please wait...")
        while count <= 30:
            try:
                RuTOS.delete_msg(self.router_hostname, count)
                count += 1
            except:
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
        try:
            sendstatus = RuTOS.send_msg(self.router_hostname, self.client_no, data.data)
            if sendstatus == "Timeout":
                rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
        except(subprocess.CalledProcessError):
            rospy.logwarn("SSH process into router has been killed.")
    
    def recv_sms(self, data):
        '''Receive incoming SMS, process it, and forward to despatcher node via ogc/from_sms topic'''
        try:
            # Read an SMS received by the air router
            msglist_raw = RuTOS.extract_msg(self.router_hostname, 1)
            self.msglist = msglist_raw.decode().splitlines()
            if 'no message' in self.msglist:
                pass
            else:
                # extract sender number (2nd word of 3rd line in msglist)
                sender = self.msglist[2].split()[1]
                # Ensure sender is whitelisted before extracting message
                if sender in self.whitelist:
                    rospy.loginfo('Command from '+ sender)
                    # msg is located on the 5th line (minus first word) of msglist. It is converted to lowercase
                    self.msg = (self.msglist[4].split(' ', 1)[1]).lower()
                    # Forward msg to air_despatcher
                    self.pub_to_despatcher.publish(self.msg)
                else:
                    rospy.logwarn('Rejected msg from unknown sender ' + sender)
                RuTOS.delete_msg(self.router_hostname, 1) # Delete the existing SMS
        except(subprocess.CalledProcessError):
            rospy.logwarn("SSH process into router has been killed.")
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        """Main function to let aircraft receive SMS commands"""
        rospy.Subscriber("ogc/to_sms", String, self.send_sms)
        message_sender = rospy.Timer(rospy.Duration(self.interval), self.recv_sms)
        rospy.spin()
        message_sender.shutdown()

if __name__=='__main__':
    run = SMSrx()
    run.client()

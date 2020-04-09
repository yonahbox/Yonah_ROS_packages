#!/usr/bin/env python3
"""
SMS_rx

Read an SMS from Ground Control, extract and process the message inside, and send
necessary commands to the aircraft.

Prerequisite: Please ensure the GCS number (GCS_no) in air.launch is correct

Available commands:

- Arm the aircraft: "arm"
- Disarm the aircraft: "disarm"
- Mode change: "mode <flight mode>"
- Set a specific waypoint number in an already-loaded mission file: "wp set <wp number>"
- Load a waypoint file that is stored in the companion computer: "wp load < absolute path to waypoint file>"
- Regular SMS update functions:
    - Activate regular SMS updates: "sms true"
    - Deactivate regular SMS updates: "sms false"
    - Set long time intervals between each update: "sms long" (on bootup, interval is long by default)
    - Set short time intervals between each update: "sms short"
- On-demand SMS update functions:
    - Request one SMS from the aircraft: "ping"
    - Request flight mode: "ping mode"
    - Request aircraft status text messages: "ping msg"
    - Request vibration levels in x, y and z axes: "ping vibe"
    - Request total number of clipping events in x, y and z axes: "ping clipping"
    - Request for statustext messages to be sent to GCS: "statustext on"
    - Request for statustext messages to not be sent to GCS: "statustext off"

Commands are not case sensitive

Lau Yan Han and Yonah, March 2020
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
        self.router_hostname = rospy.get_param("~router_hostname","root@192.168.1.1") # Hostname and IP of onboard router
        self.whitelist = set() # set of whitelisted numbers
        self.msglist = "" # Raw message extracted by router (see https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index)
        self.msg = "" # Actual message sent by GCS. Located on 5th line of msglist

        # Initialize all ROS node and topic communications
        # Make sure all services below are whitelisted in apm_pluginlists.yaml!
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')
        self.pub_to_despatcher = rospy.Publisher('ogc/from_sms', String, queue_size = 5)
        rospy.init_node('SMS_rx', anonymous=False)
        self.rate = rospy.Rate(2)
        
        # Security and safety measures
        self.populatewhitelist()
        self.purge_residual_sms()

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
    
    ############################
    # "Main" function
    ############################
    
    def client(self):
        """Main function to let aircraft receive SMS commands"""
        while not rospy.is_shutdown():
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
            
            self.rate.sleep()

if __name__=='__main__':
    run = SMSrx()
    run.client()

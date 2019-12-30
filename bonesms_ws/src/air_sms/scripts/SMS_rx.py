#!/usr/bin/env python3
"""
Read an SMS from Ground Control, extract and process the message inside, and send
necessary commands to the aircraft.

Prerequisite: Please ensure the GCS number (GCS_no) in air.launch is correct

Message format:
- Arm the aircraft: "arm"
- Disarm the aircraft: "disarm"
- Mode change: "mode <flight mode in lowercase letters>"
- Activate regular SMS sending from aircraft: "sms true"
- Deactivate regular SMS sending from aircraft: "sms false"
- Request one SMS from the aircraft: "ping"
- Set long time intervals between each SMS message: "sms long" (on bootup, interval is long by default)
- Set short time intervals between each SMS message: "sms short"
- Set a specific waypoint number in an already-loaded mission file: "wp set <wp number>"
- Load a waypoint file that is stored in the companion computer: "wp load <path to waypoint file>"
Commands are not case sensitive
"""

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
from std_msgs.msg import String

# Local
import RuTOS

class SMSrx():
    
    ############################
    # Initialization
    ############################

    def __init__(self):
        self.router_hostname = "root@192.168.1.1" # Hostname and IP of onboard RUT router
        self.whitelist = set() # set of whitelisted numbers, initialized as an empty set
        self.msglist = "" # Incoming msg extracted directly from RUT with the format: https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index
        self.msg = "" # actual message that was sent by the GCS. Located on the 5th line of msglist

        # Subscribe to all necessary mavros topics, prepare to publish to SMS_tx node, and initialize SMS_rx node
        # Make sure all services below are whitelisted in apm_pluginlists.yaml
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.wait_for_service('mavros/mission/set_current')
        self.sms_sender = rospy.Publisher('sendsms', String, queue_size = 5)
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
                self.whitelist.add(line[:-1]) # remove whitespace at end of line
        rospy.loginfo(self.whitelist)

    def purge_residual_sms(self):
        """
        Clear the router of SMSes (capped at 30 for the RUT955) on bootup. This prevents the situation
        where an SMS is received when the air router was off, and is acted upon the moment the router
        switches on (very dangerous if that SMS is an arm msg!)
        """
        # The bad thing about this implementation is that it increases boot time by 30 seconds (1 second per SMS deleted)
        count = 1
        rospy.loginfo("Purging residual SMS, please wait...")
        while count <= 30:
            try:
                RuTOS.delete_msg(self.router_hostname, count)
                count += 1
            except:
                continue
        rospy.loginfo("Purge complete!")

    #####################################
    # Interaction with ROS Services/Nodes
    #####################################
    
    def checkArming(self):
        """Check for Arm/Disarm commands from Ground Control"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        if self.msg == "disarm":
            rospy.loginfo('DISARM')
            arm(0)
        if self.msg == "arm":
            rospy.loginfo('ARM')
            arm(1)

    def checkMode(self):
        """Check for Mode change commands from Ground Control"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        # Message structure: mode <flight mode>; extract 2nd word to get flightmode
        if 'mode' in self.msg:
            mode_command = self.msg.split()[1]
            rospy.loginfo(mode_command)
            mode(custom_mode = mode_command)
    
    def check_SMS_or_ping(self):
        """
        Handle all services related to sending of SMS to Ground Control
        If SMS sending is required, instruct SMS_tx (through the sendsms topic) to do it
        """
        if "sms" in self.msg or self.msg == "ping":
            self.sms_sender.publish(self.msg)

    def checkMission(self):
        """Check for mission/waypoint commands from Ground Control"""
        if "wp" in self.msg:
            if "wp set" in self.msg:
                # Message structure: wp set <seq_no>, hence extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = self.msg.split()[2]
                wp_set(wp_seq = int(seq_no))            
            elif "wp load" in self.msg:            
                # Message structure: wp load <wp file name>; extract 3rd word to get wp file
                # Assume that wp file is located in root directory of Beaglebone
                wp_file = self.msg.split()[2]
                subprocess.call(["rosrun", "mavros", "mavwp", "load", "/home/ubuntu/%s"%(wp_file)], shell=False)
    
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
                # if no message from sender, then just skip
                if 'no message' in self.msglist:
                    pass
                else:
                    # extract sender number (2nd word of 3rd line in msglist)
                    sender = self.msglist[2].split()[1]
                    # Ensure sender is whitelisted. If sender is whitelisted, extract the message
                    if sender in self.whitelist:
                        rospy.loginfo('Command from '+ sender)
                        # msg is located on the 5th line (minus the first word) of msglist. It is converted to lowercase
                        self.msg = (self.msglist[4].split(' ', 1)[1]).lower()
                        # Run through a series of checks to see what command should be sent to aircraft
                        self.checkArming()
                        self.check_SMS_or_ping()
                        self.checkMode()
                        self.checkMission()
                    else:
                        rospy.logwarn('Rejected msg from unknown sender ' + sender)
                    RuTOS.delete_msg(self.router_hostname, 1) # Delete the existing SMS
            
            except(subprocess.CalledProcessError):
                rospy.logwarn("SSH process into router has been killed.")
            
            except(rospy.ServiceException):
                rospy.logwarn("Service call failed")
            
            self.rate.sleep()

if __name__=='__main__':
    run = SMSrx()
    run.client()

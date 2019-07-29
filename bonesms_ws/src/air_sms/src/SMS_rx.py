#!/usr/bin/env python3
"""
Read an SMS from Ground Control, extract and process the message inside, and send
necessary commands to the aircraft.

Prerequesite: Please ensure the GCS number (GCS_no) in SMS_tx.launch is correct

Message format:
- Arm the aircraft: "arm"
- Disarm the aircraft: "disarm"
- Mode change: "mode <flight mode in lowercase letters>"
- Activate SMS sending from aircraft: "sms true"
- Deactivate SMS sending from aircraft: "sms false"
- Set long time intervals between each SMS message: "sms long" (on bootup, interval is long by default)
- Set short time intervals between each SMS message: "sms short"
Commands are not case sensitive
"""

import rospy
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import subprocess

class SMSrx():
    
    def __init__(self):
        self.whitelist = set() # set of whitelisted numbers, initialized as an empty set
        self.msglist = "" # Incoming msg extracted directly from RUT, consisting of 5 lines that is specified here: https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index
        self.msg = "" # actual message that was sent by the GCS. It is located on the 5th line of msglist

        # Subscribe to all necessary mavros topics, prepare to publish to SMS_tx node, and initialize SMS_rx node
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        self.sms_sender = rospy.Publisher('sendsms', String, queue_size = 5)
        rospy.init_node('SMS_rx', anonymous=False)
        self.rate = rospy.Rate(2)
        
        # Security and safety measures
        self.populatewhitelist()
        self.purge_residual_sms()

    def populatewhitelist(self):
        """Fill up whitelisted numbers. Note that whitelist.txt must be in same folder as this script"""
        textfile = rospy.get_param("~whitelist", "src/whitelist.txt")
        with open (textfile, "r") as reader:
            for line in reader:
                self.whitelist.add(line[:-1]) # remove whitespace at end of line
        rospy.loginfo(self.whitelist)

    def purge_residual_sms(self):
        """
        Removes SMSes entering the air router for 5 seconds after bootup. This prevents the situation
        where an SMS might have been sent when the air router was off, and the air router acts
        on the SMS the moment it switches on (which can be very dangerous if that SMS is an arm msg!)
        
        To-do: Replace this with a more robust system to delete any messages that were delivered before
        air router startup
        """
        start_time = rospy.get_time()
        cur_time = rospy.get_time()
        rospy.loginfo("Purging residual SMS, please wait...")
        while (cur_time - start_time) < 5.0:
            try:
                subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -d 1"], shell=False)
                cur_time = rospy.get_time()
            except:
                cur_time = rospy.get_time()
                continue
        rospy.loginfo("Purge complete!")

    def checkArming(self):
        """Check for Arm/Disarm commands from sender"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        if self.msg == "disarm":
            rospy.loginfo('DISARM')
            arm(0)

        if self.msg == "arm":
            rospy.loginfo('ARM')
            arm(1)

    def checkMode(self):
        """Check for Mode change commands from sender"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Message structure: mode <flight mode>; hence extract the 2nd word to get flightmode
        if 'mode' in self.msg:
            mode_command = self.msg.split()[1]
            rospy.loginfo(mode_command)
            mode(custom_mode = mode_command)
    
    def check_SMS(self):
        """
        Check if SMS_tx should send out SMSes, and whether the
        interval between each SMS sent should be short or long (defaults to long on bootup)
        """
        if "sms" in self.msg:
            self.sms_sender.publish(self.msg)
    
    def client(self):
        """Main function to let aircraft receive SMS commands"""
    
        # Main loop
        while not rospy.is_shutdown():
            try:
                # Read an SMS received by the air router (stored in ReadCMD as a string)
                ReadCMDraw = subprocess.check_output(["ssh", "root@192.168.1.1", "gsmctl -S -r 1"], shell=False)
                self.msglist = ReadCMDraw.decode().splitlines()

                # if no message from sender, then just skip
                if 'no message' in self.msglist:
                    pass
            
                else:
                    # extract sender number (2nd word of 3rd line in msglist)
                    sender = self.msglist[2].split()[1]
                
                    # Ensure sender is whitelisted. If sender is whitelisted, extract the message
                    # msg is located on the 5th line (minus the first word) of msglist. It is converted to lowercase automatically
                    # Then run through a series of checks to see what command should be sent to aircraft
                    if sender in self.whitelist:
                        rospy.loginfo('Command from '+ sender)
                        self.msg = (self.msglist[4].split(' ', 1)[1]).lower()
                        self.checkArming()
                        self.check_SMS()
                        self.checkMode()

                    else:
                        rospy.logwarn('Rejected msg from unknown sender ' + sender)

                    subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -d 1"], shell=False) # Delete the existing SMS message
            
            except(subprocess.CalledProcessError):
                rospy.logwarn("SSH process into router has been killed.")
            
            except(rospy.ServiceException):
                rospy.logwarn("Service call failed")
            
            self.rate.sleep()

if __name__=='__main__':
    run = SMSrx()
    run.client()

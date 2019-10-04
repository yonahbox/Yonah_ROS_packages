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
- Load a waypoint file that is stored in the companion computer: "wp load <absolute path to waypoint file>"
Commands are not case sensitive
"""

import rospy
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointSetCurrent
import subprocess

class SMSrx():
    
    def __init__(self):
        self.whitelist = set() # set of whitelisted numbers, initialized as an empty set
        self.msglist = "" # Incoming msg extracted directly from RUT, consisting of 5 lines that is specified here: https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index
        self.msg = "" # actual message that was sent by the GCS. It is located on the 5th line of msglist

        # Subscribe to all necessary mavros topics, prepare to publish to SMS_tx node, and initialize SMS_rx node
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
        textfile = rospy.get_param("~whitelist", "src/whitelist.txt")
        with open (textfile, "r") as reader:
            for line in reader:
                self.whitelist.add(line[:-1]) # remove whitespace at end of line
        rospy.loginfo(self.whitelist)

    def purge_residual_sms(self):
        """
        Clear the router of SMSes (capped at 30 for the RUT955) upon bootup. This prevents the situation
        where an SMS might have been sent when the air router was off, and the air router acts
        on the SMS the moment it switches on (which can be very dangerous if that SMS is an arm msg!)
        
        To-do: Replace this with a more robust system to delete any messages that were delivered before
        air router startup
        """
        # The bad thing about this new implementation is that it increases the boot time of the nodes
        # by 30 seconds (1 second for each SMS deleted)
        count = 1
        rospy.loginfo("Purging residual SMS, please wait...")
        while count <= 30:
            try:
                subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -d '%s'"%(str(count))], shell=False)
                count += 1
            except:
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
    
    def check_SMS_or_ping(self):
        """
        Check if SMS_tx should send out SMSes, and whether the
        interval between each SMS sent should be short or long (defaults to long on bootup)
        Also handle ping request: If GCS sends "ping" command, then instruct SMS_tx to send one message
        """
        if "sms" in self.msg or self.msg == "ping":
            self.sms_sender.publish(self.msg)

    def checkMission(self):
        """Check for waypoint file loading, or waypoint sequence set commands from sender"""
        
        if "wp" in self.msg:

            if "wp set" in self.msg:

                # Message structure: wp set <seq_no>, hence extract the 3rd word to get seq no
                wp_set = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
                seq_no = self.msg.split()[2]
                wp_set(seq_no)
            
            elif "wp load" in self.msg:
            
                # Message structure: wp load <wp file name>, hence extract 3rd word to get wp file name
                # In this implementation, assume that wp file is located in root directory of Beaglebone
                wp_file = self.msg.split()[2]
                subprocess.call(["rosrun", "mavros", "mavwp", "load", "~/%s"%(wp_file)], shell=False)
    
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
                        self.check_SMS_or_ping()
                        self.checkMode()
                        self.checkMission()

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

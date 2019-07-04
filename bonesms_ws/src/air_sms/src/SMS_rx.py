#!/usr/bin/env python3
"""
Read an SMS from Ground Control, extract and process the message inside, and send
necessary commands to the aircraft.

Message format:
- Arm the aircraft: "arm"
- Disarm the aircraft: "disarm"
- Mode change: "mode <flight mode in lowercase letters>"
- Activate SMS sending from aircraft: "sms true"
- Deactivate SMS sending from aircraft: "sms false"
Commands are not case sensitive
"""

# Possible bug: need to clear existing SMS msg inside the air router before starting up this script

import sys
import rospy
import os
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
import subprocess
import roslaunch

class SMSrx():
    
    def __init__(self):
        self.whitelist = set() # set of whitelisted numbers, initialized as an empty set
        self.msglist = "" # Incoming msg extracted directly from RUT, consisting of 5 lines that is specified here: https://wiki.teltonika.lt/view/Gsmctl_commands#Read_SMS_by_index
        self.msg = "" # actual message that was sent by the GCS. It is located on the 5th line of msglist

        # "Global" variables to be modified each time check_SMS_true_false is called
        self.sms_flag = False
        self.process = None
        self.launch = None

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/set_mode')
        rospy.init_node('SMS_rx', anonymous=False)
        self.rate = rospy.Rate(2)
        self.node = roslaunch.core.Node('air_sms','SMS_tx.py')
        self.populatewhitelist()

    def populatewhitelist(self):
        """Fill up whitelisted numbers. Note that whitelist.txt must be in same folder as this script"""
        textfile = rospy.myargv(argv=sys.argv)[1]
        with open (textfile, "r") as reader:
            for line in reader:
                self.whitelist.add(line[:-1]) # remove whitespace at end of line
        print(self.whitelist)

    def checkArming(self):
        """Check for Arm/Disarm commands from sender"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        if self.msg == "disarm":
            print('DISARM')
            arm(0)

        if self.msg == "arm":
            print('ARM')
            arm(1)

    def checkMode(self):
        """Check for Mode change commands from sender"""
        mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        # Message structure: mode <flight mode>; hence extract the 2nd word to get flightmode
        if 'mode' in self.msg:
            mode_command = self.msg.split()[1]
            print (mode_command)
            mode(custom_mode = mode_command)
    
    def check_SMS_true_false(self):
        """Check if air router should send out SMSes"""
        if self.msg == "sms true":
            self.launch = roslaunch.scriptapi.ROSLaunch()
            self.launch.start()
            self.process = self.launch.launch(self.node)
            print(self.process.is_alive())
            self.sms_flag = True

        if self.msg == "sms false":
            # SMS:False should only be called if SMS_tx is already running
            if not self.sms_flag:
                pass
            else:
                self.process.stop()
                self.launch.stop()
    
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
                        print ('Command from '+ sender)
                        self.msg = (self.msglist[4].split(' ', 1)[1]).lower()
                        self.checkArming()
                        self.check_SMS_true_false()
                        self.checkMode()

                    else:
                        print ('Rejected msg from unknown sender ' + sender)

                    subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -d 1"], shell=False) # Delete the existing SMS message
            
            except(subprocess.CalledProcessError):
                print("SSH process into router has been killed.")
            
            except(rospy.ServiceException):
                print("Service call failed")
            
            self.rate.sleep()

if __name__=='__main__':
    if len(rospy.myargv(argv=sys.argv)) < 2:
        print("Usage: rosrun air_sms SMS_rx.py <path to whitelist>")
    else:
        run = SMSrx()
        run.client()

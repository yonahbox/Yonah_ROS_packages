#!/usr/bin/env python3
import sys
import rospy
import os
from mavros_msgs.srv import CommandBool
import subprocess
import roslaunch

class SMSrx():
    
    def __init__(self):
        self.whitelist = set() # set of whitelisted numbers, initialized as an empty set
        self.ReadCMD = ""

        # "Global" variables to be modified each time check_SMS_true_false is called
        self.sms_flag = False
        self.process = None
        self.launch = None

        rospy.wait_for_service('mavros/cmd/arming')
        rospy.init_node('SMS_rx', anonymous=False)
        self.rate = rospy.Rate(2)
        self.node = roslaunch.core.Node('yonah','SMS_tx.py')
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
        
        if 'Arm:False' in self.ReadCMD:
            print('DISARM')
            arm(0)

        if 'Arm:True' in self.ReadCMD:
            print('ARM')
            arm(1)
    
    def check_SMS_true_false(self):
        """Check if air router should send out SMSes"""
        if 'SMS:True' in self.ReadCMD:
            self.launch = roslaunch.scriptapi.ROSLaunch()
            self.launch.start()
            self.process = self.launch.launch(self.node)
            print(self.process.is_alive())
            self.sms_flag = True

        if 'SMS:False' in self.ReadCMD:
            if not self.sms_flag: # SMS:False should only be called if SMS_tx is already running
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
                self.ReadCMD = ReadCMDraw.decode()
                #print(ReadCMD)

                if 'no message' in self.ReadCMD: # if no message from sender, then just skip
                    pass
            
                else:
                    sender = (self.ReadCMD.splitlines()[2]).split()[1] # extract sender number (2nd word of 3rd line in ReadCMD
                
                    if sender in self.whitelist: # Ensure sender is whitelisted
                        print ('Command from '+ sender)

                        # If sender is whitelisted, run through a series of checks to decipher and execute the command
                        self.checkArming()
                        self.check_SMS_true_false()

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
        raise Exception("You must specify the whitelist, e.g. rosrun yonah SMS_rx.py <path to whitelist>")
    run = SMSrx()
    run.client()

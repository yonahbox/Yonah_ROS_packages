#!/usr/bin/env python
import sys
import rospy
import os
from mavros_msgs.srv import CommandBool
import subprocess
import roslaunch

class SMSrx():
    
    whitelist = set() # set of whitelisted numbers, initialized as an empty set
    ReadCMD = ""

    def populatewhitelist(self):
        """Fill up whitelisted numbers. Note that whitelist.txt must be in same folder as this script"""
        textfile = rospy.myargv(argv=sys.argv)[1]
        with open (textfile, "r") as reader:
            for line in reader:
                self.whitelist.add(line[:-1]) # remove whitespace at end of line
        print self.whitelist

    def checkArming(self):
        """Check for Arm/Disarm commands from sender"""
        arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        
        if 'Arm:False' in self.ReadCMD:
            print 'DISARM'
            arm(0)

        if 'Arm:True' in self.ReadCMD:
            print 'ARM'
            arm(1)
    
    def check_SMS_true_false(self, node):
        """Check if air router should send out SMSes"""
        if 'SMS:True' in self.ReadCMD:
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            process = launch.launch(node)
            print process.is_alive()

        if 'SMS:False' in self.ReadCMD:
            process.stop()
            launch.stop()
    
    def client(self):
        """Main function to let aircraft receive SMS commands"""
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.init_node('SMS_rx', anonymous=False)
        rate = rospy.Rate(2)
        node = roslaunch.core.Node('yonah','SMS_tx.py')
        #first = False
    
        self.populatewhitelist()
    
        while not rospy.is_shutdown():
            try:
                arm = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                self.ReadCMD = subprocess.check_output(["ssh", "root@192.168.1.1", "gsmctl -S -r 1"], shell=False)
                #print(ReadCMD)

                if 'no message' in self.ReadCMD: # if no message from sender, then just skip
                    pass
            
                else:
                    sender = (self.ReadCMD.splitlines()[2]).split()[1] # extract sender number (2nd word of 3rd line in ReadCMD
                
                    if sender in self.whitelist: # Ensure sender is whitelisted
                        print ('Command from '+ sender)

                        # If sender is whitelisted, run through a series of checks to decipher and execute the command
                        self.checkArming()
                        self.check_SMS_true_false(node)

                    else:
                        print ('Rejected msg from unknown sender ' + sender)

                    subprocess.call(["ssh", "root@192.168.1.1", "gsmctl -S -d 1"], shell=False)
 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            rate.sleep()

if __name__=='__main__':
    if len(rospy.myargv(argv=sys.argv)) < 2:
        raise Exception("You must specify the whitelist, e.g. rosrun yonah SMS_rx.py <path to whitelist>")
    run = SMSrx()
    run.client()

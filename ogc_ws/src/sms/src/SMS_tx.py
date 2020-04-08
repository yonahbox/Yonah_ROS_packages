#!/usr/bin/env python3

'''
SMS_tx

Obtain message from despatcher nodes and send it as an SMS

'''

# Standard Library
import subprocess

# ROS/Third-Party
import rospy
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import State
from mavros_msgs.msg import StatusText
from mavros_msgs.msg import Vibration
from std_msgs.msg import String

# Local
import RuTOS

class SMStx():

    ############################
    # Initialization
    ############################
    
    def __init__(self):
        '''Initialize all message entries'''
        rospy.init_node('SMS_tx', anonymous=False)
        self.router_hostname = rospy.get_param("~router_hostname","root@192.168.1.1") # Hostname and IP of onboard router
        self.GCS_no = rospy.get_param("~GCS_no", "12345678") # GCS phone number

    #########################################
    # Handle sending of SMS to Ground Control
    #########################################
    
    def send_sms(self, data):
        '''
        Send msg from despatcher node (over ogc/to_sms topic) as an SMS
        '''
        rospy.loginfo("Sending SMS: " + data.data)
        try:
            sendstatus = RuTOS.send_msg(self.router_hostname, self.GCS_no, data.data)
            if sendstatus == "Timeout":
                rospy.logerr("Timeout: Aircraft SIM card isn't responding!")
        except(subprocess.CalledProcessError):
            rospy.logwarn("SSH process into router has been killed.")


    ############################
    # "Main" function
    ############################
    
    def prepare(self):
        '''
        Main function to subscribe to all mavros topics and send SMS messages (if requested by SMS_rx node)
        If regular payload is activated, messages are sent with a time interval (seconds) specified by self.interval
        '''
        rospy.Subscriber("ogc/to_sms", String, self.send_sms)
        rospy.spin()

if __name__=='__main__':
    run = SMStx()
    run.prepare()
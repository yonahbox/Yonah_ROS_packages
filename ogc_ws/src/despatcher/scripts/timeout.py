#!/usr/bin/env python3

import rospy
from std_msgs.msg import String 

from despatcher.msg import LinkMessage

class MessageTimer():
    def __init__(self, message, id):
        self.id = id
        self.message = message
        self.timeout = [10, 20]
        self.status = "Not Sent"
        self._watchdog = self.timeout

    def advance(self):
        pass

    def incoming_message(self, data):
        print(data)
        print("Test")
        pass

    def countdown(self, data):
        pass

    def start_first_timer(self):
        pass

    '''Main Function'''
    def client(self):
        watchdog = rospy.Timer(rospy.Duration(1), self.countdown)
        rospy.spin()
        watchdog.shutdown()

def incoming_message(data):
    print(data)

class Manager():
    def __init__(self):
        rospy.init_node('timeout', anonymous=False)
        self.registered_id = []
    
    def watcher(self):
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.incoming_message)
        rospy.spin()
    
    def incoming_message(self, data):
        if data.id not in self.registered_id:
            self.registered_id.append(data.id)
            msg = MessageTimer(data.message, data.id)
            msg.client()

        print(self.registered_id)
        print(data)

if __name__=='__main__':
    run = Manager()
    run.watcher()

#!/usr/bin/env python3

import rospy
from std_msgs.msg import String 

from despatcher.msg import LinkMessage

class MessageTimer():
    def __init__(self, message, id):
        self.id = id
        self.message = message
        self.timeout = [5, 10]
        self.status = 0 # status: 0 pending, 1 sent through links, 2 acknowledged
        self._watchdog = self.timeout

    def advance(self):
        self.reset_timer()
        self.client()

    def reset_timer(self):
        self._watchdog = self.timeout

    def countdown(self, data):
        self._watchdog[self.status] -= 1
        if self._watchdog[self.status] <= 0:
            rospy.logwarn("TIMER HAS RUN OUT")
            rospy.logwarn(self.status_checker())
            self.status = -1
            self.stop_timer()

    def stop_timer(self):
        self.watcher.shutdown()
            
    def status_checker(self):
        if self.status == 0:
            return "PENDING"
        elif self.status == 1:
            return "SINGLE TICK"
        elif self.status == 2:
            return "DOUBLE TICK"
        elif self.status == -1:
            return "MESSAGE EXPIRED"
        else:
            return "STATUS UNDEFINED"

    '''Main Function'''
    def client(self):
        self.watcher = rospy.Timer(rospy.Duration(1), self.countdown)

class Manager():
    def __init__(self):
        rospy.init_node('timeout', anonymous=False)
        self.messages = {}
    
    def watcher(self):
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.sent_commands)
        # Put another subscriber in that listens to messages sent to rqt
        rospy.spin()
    
    def sent_commands(self, data):
        if data.id not in self.messages:
            self.messages[data.id] = MessageTimer(data.data, data.id)
            self.messages[data.id].client()
        else:
            self.messages[data.id].status += 1
            self.messages[data.id].stop_timer()
            self.messages[data.id].client()

if __name__=='__main__':
    run = Manager()
    run.watcher()

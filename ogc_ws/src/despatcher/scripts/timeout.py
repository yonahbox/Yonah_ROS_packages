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

    def countdown(self, data):
        # rospy.loginfo("COUNTDOWN STARTING")
        self._watchdog[self.status] -= 1
        if self._watchdog[self.status] <= 0:
            self.status = -1
    def stop_timer(self):
        self.watcher.shutdown()

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
            # rospy.logwarn("CREATING NEW FIELD")
            self.messages[data.id] = MessageTimer(data.data, data.id)
            self.messages[data.id].client()
        elif self.messages[data.id].status == -1:
            rospy.logwarn("DROPPED MESSAGE")
            return 0
        else:
            # rospy.logwarn("MODIFYING OLD FIELD")
            self.messages[data.id].status += 1
            self.messages[data.id].stop_timer()
            if self.messages[data.id].status == 2:
                rospy.logwarn("MESSAGE IS SUCCESSFULLY SENT")
                return 0
            self.messages[data.id].client()

if __name__=='__main__':
    run = Manager()
    run.watcher()

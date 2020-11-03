#!/usr/bin/env python3

import rospy
from std_msgs.msg import String 

from despatcher.msg import LinkMessage

class MessageTimer():
    def __init__(self, message, message_id):
        self.message_id = message_id
        self.message = message
        self.timeout = [5, 10]
        self.status = 0 # status: 0 pending, 1 sent through links, 2 acknowledged
        self._watchdog = self.timeout

    def countdown(self, data):
        rospy.loginfo("COUNTDOWN STARTING")
        self._watchdog[self.status] -= 1
        if self._watchdog[self.status] <= 0:
            self.stop_timer()
            send_to_rqt(self.message_id, "Command " + self.message + " with message ID " + str(self.message_id) + " has failed to send.\n\nLast status: " + str(self.status))
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
        if data.id > 50:
            data.id -= 50
        
        if data.id not in self.messages:
            rospy.logwarn("CREATING NEW FIELD")
            self.messages[data.id] = MessageTimer(data.data, data.id)
            self.messages[data.id].client()

        elif self.messages[data.id].status == -1:
            rospy.logwarn("DROPPED MESSAGE")
            return 0

        else:
            self.messages[data.id].status += 1
            self.messages[data.id].stop_timer()
            if self.messages[data.id].status == 2:
                rospy.logwarn("MESSAGE IS SUCCESSFULLY SENT")
                self.messages[data.id].status == 3
                return 0
            self.messages[data.id].client()

def send_to_rqt(message_id, data):
    message = LinkMessage()
    message.id = message_id
    message.data = data
    message.guid = 0 # 0 is for guid for non-A2G message
    pub_to_rqt.publish(message)

def convert_ack2 (data):
    data = data.split(" ")
    message = LinkMessage()
    message.id = data[0]
    message.data = data[1]
    return message

# call using func.counter to get a number
def count(func):
    def wrapper(*args, **kwargs):
        wrapper.counter += 1    # executed every time the wrapped function is called
        return func(*args, **kwargs)
    wrapper.counter = 1         # executed only once in decorator definition time
    return wrapper

@count
def func():
    pass

if __name__=='__main__':
    run = Manager()
    pub_to_rqt = rospy.Publisher("ogc/feedback_to_rqt", LinkMessage, queue_size = 5)
    run.watcher()

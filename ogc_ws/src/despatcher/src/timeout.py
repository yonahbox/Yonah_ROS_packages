#!/usr/bin/env python3
'''
timeout: A timeout tracker for each command message sent from rqt

Copyright (C) 2020 Dani Purwadi and Yonah (yonahbox@gmail.com)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''
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
        rospy.logwarn("COUNTDOWN STARTING: " + str(self._watchdog[self.status]))
        self._watchdog[self.status] -= 1
        if self._watchdog[self.status] == 0:
            self.stop_timer()
            data = "Command " + self.message + " with message ID " + str(self.message_id) + " has failed to send.\n\nLast status: " + str(self.status)
            send_to_rqt(self.message_id, data)
            self.status = -1
            
        elif self._watchdog[self.status] < 0:
            rospy.logerr("MESSAGE STATUS IS NEGATIVE")

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
        rospy.Subscriber("ogc/to_timeout", LinkMessage, self.sent_commands)
        # Put another subscriber in that listens to messages sent to rqt
        rospy.spin()

    def sent_commands(self, data):
        commands = ["sms", "statustext", "arm", "mode", "wp"]
        if data.uuid not in self.messages:
            rospy.logwarn(data)
            rospy.logwarn("CREATING NEW FIELD")
            self.messages[data.uuid] = MessageTimer(data.data, data.uuid)
            self.messages[data.uuid].client()
        
        elif data.data in commands:
            rospy.logerr("REWRITING OLD FIELD")
            self.messages[data.uuid] = MessageTimer(data.data, data.uuid)
            self.messages[data.uuid].client()

        else:
            rospy.logwarn("-----------" + str(data.data))
            if data.data == "pending":
                rospy.logwarn("MESSAGE IS PENDING")
                message = "Message pending"
                send_to_rqt(data.uuid, message)

            elif data.data == "single tick":
                self.messages[data.uuid].status = 1
                self.messages[data.uuid].stop_timer()
                self.messages[data.uuid].client()

            elif data.data == "double tick":
                self.messages[data.uuid].status = 2
                self.messages[data.uuid].stop_timer()
                self.messages[data.uuid].client()
                rospy.loginfo("MESSAGE IS SUCCESSFULLY SENT")
                
            else:
                rospy.logerr("Unknown message received: " + str(data))

def send_to_rqt(message_id, data):
    message = LinkMessage()
    message.id = message_id
    message.data = data
    message.uuid = 0 # 0 is for uuid for non-A2G message
    pub_to_rqt.publish(message)

if __name__=='__main__':
    rospy.logerr("name is called")
    run = Manager()
    pub_to_rqt = rospy.Publisher("ogc/feedback_to_rqt", LinkMessage, queue_size = 5)
    run.watcher()

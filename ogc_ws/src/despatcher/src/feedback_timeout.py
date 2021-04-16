#!/usr/bin/env python3
'''
feedback_timeout: A timeout tracker for each command message sent from rqt

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
    '''Create class for every new message'''
    def __init__(self, message, message_id):
        self.message_id = message_id
        self.message = message
        self.timeout = [10, 10, 10] # timeout for acknowledged is 0
        self.status = 0 # status: 0 pending, 1 sent through links, 2 acknowledged
        self._watchdog = self.timeout

    def countdown(self, data):
        self._watchdog[self.status] -= 1 # perform countdown
        if self._watchdog[self.status] == 0: # stop when reaches 0
            self.stop_timer()
            data = "Command " + self.message + " with message ID " + str(self.message_id) + " has failed to send.\n\nLast status: " + str(self.status)
            send_to_rqt(self.message_id, data)
            self.status = -1
            
        elif self._watchdog[self.status] < 0:
            rospy.logerr("rqt: Message status is negative")

    def stop_timer(self):
        '''Stop timer'''
        self.watcher.shutdown()

    '''Main Function'''
    def client(self):
        self.watcher = rospy.Timer(rospy.Duration(1), self.countdown)

class Manager():
    '''Main class that handles the incoming information'''
    def __init__(self):
        rospy.init_node('feedback_timeout', anonymous=False)
        self.messages = {} # Stores all the messages as MessageTimer classes
    
    def watcher(self):
        '''Watches incoming data from the topics and redirect to sent_commands'''
        rospy.Subscriber("ogc/to_despatcher", LinkMessage, self.sent_commands)
        rospy.Subscriber("ogc/to_timeout", LinkMessage, self.sent_commands)
        rospy.spin()

    def sent_commands(self, data):
        '''Main function that processes the data'''
        commands = ["sms", "statustext", "arm", "mode", "wp", "disarm", "syncthing"]
        if data.uuid not in self.messages and data.data.split()[0] in commands:
            self.messages[data.uuid] = MessageTimer(data.data, data.uuid)
            self.messages[data.uuid].client()
        
        elif data.data.split()[0] in commands:
            self.messages[data.uuid] = MessageTimer(data.data, data.uuid)
            self.messages[data.uuid].client()

        else:
            if data.data == "pending":
                message = "Message pending"
                send_to_rqt(data.uuid, message)

            elif data.data == "single tick":
                self.messages[data.uuid].status = 1
                self.messages[data.uuid].stop_timer()
                self.messages[data.uuid].client()

            elif data.data == "double tick":
                self.messages[data.uuid].status = 2
                self.messages[data.uuid].stop_timer()
                
            else:
                # Ignore if it is a mission command as we don't do timeout for mision commands
                if "mission" in data.data:
                    return
                rospy.logerr("rqt: Unknown message received: " + str(data))

def send_to_rqt(message_id, data):
    '''Handles sending LinkMessage to rqt'''
    message = LinkMessage()
    message.id = message_id
    message.data = data
    message.uuid = 0 # 0 is for uuid for non-A2G message
    pub_to_rqt.publish(message)

if __name__=='__main__':
    run = Manager()
    pub_to_rqt = rospy.Publisher("ogc/feedback_to_rqt", LinkMessage, queue_size = 5)
    run.watcher()

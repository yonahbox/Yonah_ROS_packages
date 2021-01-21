"""
feedback_utl.py: Module to handle all feedback message common functions

Copyright (C) 2020, Dani Purwadi and Yonah (yonahbox@gmail.com)

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
"""


import rospy
from despatcher.msg import LinkMessage

uuid = 0

def ack_converter(data, state):
    '''Converts status of a message to a recognised status'''
    if data.uuid == 0:
        return None
    else:
        message = LinkMessage()
        message.uuid = data.uuid
        message.id = 0 #@TODO change the message id to the supposed value
        if state == 0:
            message.data = "pending"
        elif state == 1:
            message.data = "single tick"
        elif state == 2:
            message.data = "double tick"
    return message

def increment():
    '''Dispense a unique uuid for every message'''
    global uuid
    uuid += 1
    if uuid > 255: # If uuid is above 255, reset the counter
        uuid = 1
    return uuid

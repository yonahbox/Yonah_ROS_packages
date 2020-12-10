#!/usr/bin/env python3

'''
headers.py: Handle msg headers
'''

# Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import rospy

PREFIX_COUNT = 4
SUFFIX_COUNT = 1

class new_msg_chk():
    '''Check whether incoming msgs are new'''
    def __init__(self, valid_id_list):
        self._prev_transmit_time = dict() # Transmit times of previous incoming msgs
        for i in valid_id_list:
            self._prev_transmit_time[i] = rospy.get_rostime().secs
    
    def is_new_msg(self, timestamp, sysid):
        '''Return true is incoming msg is a new msg from sysid'''
        if not sysid in self._prev_transmit_time:
            return False
        if timestamp < self._prev_transmit_time[sysid]:
            return False
        else:
            self._prev_transmit_time[sysid] = timestamp
            return True

######################
# Handle incoming msgs
######################

def split_headers(msg):
    '''
    Take in a msg + headers
    Return the headers along with a list of the words in the msg (split according to spaces)
    If the msg is in invalid format, return None
    '''
    try:
        msglist = msg.split()
        msgtype = str(msglist[0])
        devicetype = int(msglist[1])
        sysid = int(msglist[2])
        uuid = int(msglist[3])
        timestamp = int(msglist[-1])
        return msgtype, devicetype, sysid, uuid, timestamp, msglist[PREFIX_COUNT:-SUFFIX_COUNT]
    except:
        return None
    
######################
# Handle outgoing msgs
######################

def attach_headers(prefixes, suffixes, msg):
    '''
    Take in a list of prefixes, list of suffixes, and the msg
    Return the msg with the attached prefixes/suffixes
    If prefixes/suffixes are invalid, return None
    '''
    if len(prefixes) != PREFIX_COUNT or len(suffixes) != SUFFIX_COUNT:
        return None
    prefix_string = " ".join(str(i) for i in prefixes)
    suffix_string = " ".join(str(i) for i in suffixes)
    return prefix_string + " " + msg + " " + suffix_string
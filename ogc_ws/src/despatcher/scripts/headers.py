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

PREFIX_COUNT = 3
SUFFIX_COUNT = 1

class headerhandler():
    
    def __init__(self, max_valid_id):
        self._prev_transmit_time = list() # Transmit times of previous incoming msgs
        i = 0
        while i < max_valid_id:
            self._prev_transmit_time.append(rospy.get_rostime().secs)
            i = i + 1
    
    ######################
    # Handle incoming msgs
    ######################
    
    def is_new_msg(self, timestamp, sysid):
        '''Return true is incoming msg is a new msg from sysid. sysid assumed to start from 1'''
        if sysid > len(self._prev_transmit_time):
            return False
        if timestamp < self._prev_transmit_time[sysid - 1]:
            return False
        else:
            self._prev_transmit_time[sysid - 1] = timestamp
            return True
    
    def split_headers(self, msg):
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
            timestamp = int(msglist[-1])
            return msgtype, devicetype, sysid, timestamp, msglist[PREFIX_COUNT:-SUFFIX_COUNT]
        except:
            return None
    
    ######################
    # Handle outgoing msgs
    ######################

    def attach_headers(self, prefixes, suffixes, msg):
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
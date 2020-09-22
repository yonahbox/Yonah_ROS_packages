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
    
    def __init__(self):
        self._prev_transmit_time = rospy.get_rostime().secs
    
    ######################
    # Handle incoming msgs
    ######################
    
    def _is_new_msg(self, timestamp):
        '''Return true is incoming msg is a new msg'''
        if timestamp < self._prev_transmit_time:
            return False
        else:
            self._prev_transmit_time = timestamp
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
        prefix_string = "".join(str(i) for i in prefixes)
        suffix_string = "".join(str(i) for i in suffixes)
        return prefix_string + " " + msg + " " + suffix_string
#!/usr/bin/env python3

"""
regular.py: Module to handle regular payload logic

Copyright (C) 2020, Lau Yan Han and Yonah (yonahbox@gmail.com)

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

from despatcher.msg import RegularPayload

class RegularPayloadException(Exception):
    def __init__(self, msg):
        self.msg = msg
        super().__init__(self.msg)

    def __str__(self):
        return self.msg

########################################
# Packing/Unpacking of regular payload
########################################

# The regular payload should comprise only of short integers/characters
# with the exception of the first (R msg prefix) and last (Unix timestamp)
# Everything is standardized to big endian to keep in line with Rock 7's requirements
# Example payload: r 1 1 30 226 1 30 2 2315 102 6857 4 0 0 1591089280
struct_cmd = "> s B B B H B B B H B H B H B I"
no_of_entries = len(struct_cmd.split()[1:])

def get_compressed_len():
    '''Return length of compressed regular payload based on struct_cmd'''
    global struct_cmd
    cmd_breakdown = struct_cmd.split()[1:]
    size = 0
    for i in cmd_breakdown:
        if i == 's' or i == 'B' or i == 'b':
            size = size + 1
        elif i == 'H' or i == 'h':
            size = size + 2
        elif i == "I" or i == 'i':
            size = size + 4
        else:
            raise RegularPayloadException\
                ("Forbidden value " + i + " in struct cmd. Allowed values: s, B, b, H, h, I, i")
    return size

def convert_to_list(mo_msg):
    '''Convert regular payload msg from string to list for easy struct packing'''
    li = mo_msg.split()
    return [li[0].encode(),] + list(map(int, li[1:])) # Convert all str to int (except prefix)

def convert_to_str(mo_msg):
    '''Convert regular payload msg from list to string after struct unpacking, to standardize with other links'''
    # Example: (b'r', 1, 1, 0, 226, 0, 0, 2, 2315, 102, 6857, 0, 0, 0, 1591159898)
    string = str(mo_msg)
    string = string.replace('b\'r\'', 'r', 1) # The 'r' msg prefix is still byte encoded
    bad_char = ",()"
    for i in bad_char:
        string = string.replace(i,"") # Remove unnecessary characters
    return string

def convert_mode_to_int (mode):
        d = {
            'MANUAL': 0,
            'CIRCLE': 1,
            'STABILIZE': 2,
            'TRAINING': 3,
            'ACRO': 4,
            'FBWA': 5,
            'FBWB': 6,
            'CRUISE': 7,
            'AUTOTUNE': 8,
            'AUTO': 10,
            'RTL': 11,
            'LOITER': 12,
            'LAND': 14,
            'GUIDED': 15,
            'INITIALISING': 16,
            'QSTABILIZE': 17,
            'QHOVER': 18,
            'QLOITER': 19,
            'MANUAL': 20,
            'QLAND': 21
        }
        return d.get(mode)
########################################
# Gnd despatcher
########################################

def is_regular(sender_msgtype, entries_len):
    '''
    Check if msg is regular payload by checking the msg prefix (sender_msgtype) and number of entries within the msg
    Return True if payload is a regular payload
    '''
    global no_of_entries
    if entries_len == no_of_entries and sender_msgtype == 'r':
        return True
    return False

def convert_to_rosmsg(entries):
    '''Transform entries (msg converted to list) into a RegularPayload ros msg'''
    rosmsg = RegularPayload()
    rosmsg.header.frame_id = entries[0]
    rosmsg.is_aircraft = int(entries[1])
    rosmsg.vehicle_no = int(entries[2])
    rosmsg.airspeed = int(entries[3])
    rosmsg.alt = int(entries[4])
    rosmsg.armed = int(entries[5])
    # rosmsg.mode = int(entries[6])
    rosmsg.groundspeed = int(entries[6])
    rosmsg.lat = int(entries[7]) + float(entries[8])/10000
    rosmsg.lon = int(entries[9]) + float(entries[10])/10000
    rosmsg.throttle = float(entries[11])/10
    rosmsg.vtol = int(entries[12])
    rosmsg.wp = int(entries[13])
    rosmsg.header.stamp.secs = int(entries[14])
    return rosmsg

########################################
# air despatcher
########################################

class air_payload():

    def __init__(self):
        self.entries = {
            "airspeed": 0,
            "alt": 0,
            "arm": 0,
            "mode": 0, # new done
            "groundspeed": 0,
            "windspeed": 0, # currently no topic is changing this value
            "fuel": 0, # as of now fuel's topic hasn't been established yet
            "batt": 0, # new
            "lat1": 0,
            "lat2": 0,
            "lon1": 0,
            "lon2": 0,
            "throttle": 0,
            "wp": 0,
            "vtol": 0
            # "vibe" : 0
        }
        self.ping_entries = {
            "vibe": (0.0, 0.0, 0.0),
            "clipping": (0,0,0)
        }

    def get_mode_and_arm_status(self, data):
        '''Obtain mode and arm status from mavros/state'''
        self.entries["arm"] = int(data.armed)
        self.entries["mode"] = convert_mode_to_int(data.mode)
        self.ping_entries["mode"] = data.mode
    
    def get_VFR_HUD_data(self, data):
        '''Obtain VFR_HUD data from mavros/vfr_hud'''
        self.entries["airspeed"] = int(data.airspeed)
        self.entries["groundspeed"] = int(data.groundspeed)
        self.entries["throttle"] = int(data.throttle*10)
        self.entries["alt"] = int(data.altitude)

    def get_GPS_coord(self, data):
        '''Obtain GPS latitude and longitude from mavros/global_position/global'''
        # See https://en.wikipedia.org/wiki/Decimal_degrees for degree of precision of lat/lon
        self.entries["lat1"] = int(data.latitude)
        self.entries["lat2"] = int(10000*(data.latitude - self.entries["lat1"]))
        self.entries["lon1"] = int(data.longitude)
        self.entries["lon2"] = int(10000*(data.longitude - self.entries["lon1"]))

    def get_wp_reached(self, data):
        '''Obtain information on which waypoint has been reached'''
        self.entries["wp"] = data.wp_seq

    def get_VTOL_mode(self, data):
        '''Check whether any of the quad outputs are active, to determine if we are in VTOL mode'''
        if data.channels[4] > 1200 or data.channels[5] > 1200 or data.channels[6] > 1200\
            or data.channels[7] > 1200:
            self.entries["vtol"] = 1
        else:
            self.entries["vtol"] = 0
    
    # @TODO check whether data.windspeed is indeed correct
    def get_windspeed(self, data):
        self.entries['windspeed'] = int(data.windspeed)

    def get_vibe_status(self, data):
        '''Obtain vibration data from mavros/vibration/raw/vibration'''
        self.ping_entries["vibe"] = (round(data.vibration.x, 2), round(data.vibration.y, 2),\
            round(data.vibration.z, 2))
        self.ping_entries["clipping"] = (data.clipping[0], data.clipping[1], data.clipping[2])
        # self.entries['vibe']
    def truncate_regular_payload(self):
        '''Remove unnecessary characters from regular payload'''
        msg = str(sorted(self.entries.items())) # Sort entries and convert to string
        bad_char = ",[]()'"
        for i in bad_char:
            msg = msg.replace(i,"") # Remove unnecessary characters
        for k in self.entries.keys():
            k = k + " "
            msg = msg.replace(k,"") # Remove entry descriptions
        return msg
    
    
#!/usr/bin/env python3

'''
RuTOS: Commands to interact with RuT Teltonika routers (which use RuTOS).
See https://wiki.teltonika-networks.com/view/Gsmctl_commands for more details

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
'''

import subprocess
    
def send_msg(hostname, GCS_no, msg):
    '''Send msg to the specified GCS number'''
    subprocess.call(["ssh", "%s"%(hostname), "gsmctl -S -s '%s %s'"%(GCS_no, msg)], shell=False)
    
def extract_msg(hostname, n):
    '''Extract an incoming message in the router's nth entry and return it as a raw string'''
    msglist_raw = subprocess.check_output(["ssh", "%s"%(hostname), "gsmctl -S -r %d"%(n)], shell=False)
    return msglist_raw

def delete_msg(hostname, n):
    '''Delete the nth message in the onboard router'''
    subprocess.call(["ssh", "%s"%(hostname), "gsmctl -S -d %d"%(n)], shell=False)

def get_gps_coords(hostname):
    '''Get GPS coordinates of router'''
    subprocess.call(["gpsctl", "%s"%(hostname), "gpsctl -i -x"], shell=False)

def get_gps_alt(hostname):
    '''Get GPS altitude of router'''
    subprocess.call(["gpsctl", "%s"%(hostname), "gpsctl -a"], shell=False)

def get_gps_speed(hostname):
    '''Get GPS speed of router'''
    subprocess.call(["gpsctl", "%s"%(hostname), "gpsctl -v"], shell=False)
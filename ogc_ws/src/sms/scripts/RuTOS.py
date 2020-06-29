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

import paramiko

def hello_world_from_rutos():
    '''Sanity Checks'''
    print("My RuTOS module is alive and successfully imported")    

def start_client(ip, user):
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.connect(ip, username=user)
    return ssh

def send_msg(ssh, GCS_no, msg):
    '''Send msg to the specified GCS number'''
    _, stdout, _ = ssh.exec_command("gsmctl -S -s '%s %s'"%(GCS_no, msg))
    sendstatus = stdout.readlines()
    return sendstatus
    
def extract_msg(ssh, count):
    '''Extract an incoming message in the router's nth entry and return it as a raw string'''
    _, stdout, _ = ssh.exec_command("gsmctl -S -r %d"%(count))
    msglist = stdout.readlines()
    return msglist

def delete_msg(ssh, count):
    '''Delete the nth message in the onboard router'''
    ssh.exec_command("gsmctl -S -d %d"%(count))
    return

def blink_button(ssh):
    ssh.exec_command("gpio.sh invert DOUT1")

def button_on(ssh):
    ssh.exec_command("gpio.sh set DOUT1")

def check_button(ssh):
    _, stdout, _ = ssh.exec_command("gpio.sh get DIN1")
    button_state = int(str(stdout.readlines())[2])
    return button_state

def get_gps_coords(ssh):
    '''Get GPS coordinates of router'''
    _, stdout, _ = ssh.exec_command("gpsctl -i -x")
    coords = stdout.readlines()
    return coords

def get_gps_alt(ssh):
    '''Get GPS altitude of router'''
    _, stdout, _ = ssh.exec_command("gpsctl -a")
    alt = stdout.readlines()
    return alt

def get_gps_speed(ssh):
    '''Get GPS speed of router'''
    _, stdout, _ = ssh.exec_command("gpsctl -v")
    speed = stdout.readlines()
    return speed
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
    try:
        _, stdout, _ = ssh.exec_command("gsmctl -S -s '%s %s'"%(GCS_no, msg), timeout=5)
        sendstatus = stdout.readlines()
    except:
        sendstatus = "Timed out"
    return sendstatus
    
def extract_msg(ssh, count):
    '''Extract an incoming message in the router's nth entry and return it as a raw string'''
    try:
        _, stdout, _ = ssh.exec_command("gsmctl -S -r %d"%(count), timeout=5)
        msglist = stdout.readlines()
    except:
        msglist = "Timed out"
    return msglist

def delete_msg(ssh, count):
    '''Delete the nth message in the onboard router'''
    ssh.exec_command("gsmctl -S -d %d"%(count))
    return

def button_on(ssh):
    ssh.exec_command("gpio.sh set DOUT1")
    return

def blink_button(ssh):
    ssh.exec_command("gpio.sh invert DOUT1")
    return
    
def button_off(ssh):
    ssh.exec_command("gpio.sh set DOUT1")
    ssh.exec_command("gpio.sh invert DOUT1")
    return

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

def get_conntype(ssh):
    '''Get type of connection'''
    _, stdout, _ = ssh.exec_command("gpsctl -t")
    conntype = stdout.readlines()[0]
    return conntype

def get_rssi(ssh):
    '''Get RSSI'''
    _, stdout, _ = ssh.exec_command("gsmctl -q")
    res = stdout.readlines()[0].rstrip()
    if res == "Timeout.":
        return res
    try:
        rssi = int(res)
    except:
        return
    return rssi

def get_rsrp(ssh):
    '''Get RSRP'''
    _, stdout, _ = ssh.exec_command("gsmctl -W")
    res = stdout.readlines()[0].rstrip()
    if res == "Timeout.":
        return res
    try:
        rsrp = int(res)
    except:
        return
    return rsrp

def get_rsrq(ssh):
    '''Get RSRQ'''
    _, stdout, _ = ssh.exec_command("gsmctl -M")
    res = stdout.readlines()[0].rstrip()
    if res == "Timeout.":
        return res
    try:
        rsrq = float(res)
    except:
        return
    return rsrq

def get_sinr(ssh):
    '''Get SINR'''
    _, stdout, _ = ssh.exec_command("gsmctl -Z")
    res = stdout.readlines()[0].rstrip()
    if res == "Timeout.":
        return res
    try:
        sinr = float(res)
    except:
        return
    return sinr
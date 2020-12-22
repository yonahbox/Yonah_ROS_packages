#!/usr/bin/env python3

"""
waypoint.py: Module to handle MAVROS waypoint logic

Copyright (C) 2020, Wang Huachen and Yonah (yonahbox@gmail.com)

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

import csv
import rospy
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.srv import WaypointPush
from os import listdir, path

class WP(object):

    # Creates a csv dialect to read the waypoint file properly
    class CSVDialect(csv.Dialect):
        delimiter = '\t'
        doublequote = False
        skipinitialspace = True
        lineterminator = '\r\n'
        quoting = csv.QUOTE_NONE

    def read(self, wpfile):
        waypoints = []
        f = open(wpfile, "r")
        # Header lines look like this "QGC WPL 110"
        pastheaderline = False
        for data in csv.reader(f, self.CSVDialect):
            if not pastheaderline:
                qgc, wpl, ver = data[0].split(' ', 3)
                ver = int(ver)
                if qgc == 'QGC' and wpl == 'WPL' and (ver == 110 or ver ==120):
                    pastheaderline = True
            elif data[0].startswith("Last update"):
                break
            else:
                # Convert waypoints into Waypoint format
                waypoints.append(Waypoint(
                    is_current = bool(int(data[1])),
                    frame = int(data[2]),
                    command = int(data[3]),
                    param1 = float(data[4]),
                    param2 = float(data[5]),
                    param3 = float(data[6]),
                    param4 = float(data[7]),
                    x_lat = float(data[8]),
                    y_long = float(data[9]),
                    z_alt = float(data[10]),
                    autocontinue = bool(int(data[11]))
                ))
        return waypoints

def push_waypoints(self, waypoints):
    wppush = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
    wpset = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    if wppush(0, waypoints).success:
        if wpset(1).success:
            self._send_ack()

def get_update_time(wpfolder):
    files = listdir(wpfolder)
    updatetime = {}
    for i in files:
        # g = open(wpfolder + i, "r")
        # updatetime[i] = int(g.readlines()[-1].rstrip().split()[-1])
        updatetime[i] = int(path.getmtime(wpfolder + i))
    return updatetime

def compare_time(gndtime, airtime):
    requiredfiles = []
    for i in gndtime:
        try:
            if gndtime[i] > airtime[i]:
                requiredfiles.append(i)
        except KeyError:
            requiredfiles.append(i)
    if requiredfiles == []:
        requiredfiles = ["No update required"]
    return requiredfiles

#!/usr/bin/env python3
'''
aircraft_info: Part of RQt that shows detailed information about each active aircrafts

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

import os
import rospkg
import __main__

from python_qt_binding import loadUi
from PyQt5.QtWidgets import *
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from checklist_window import ChecklistWindow

# File is still changing rapidly and dynamically, hence comments might not be accurate
# @TODO change the variable names. As of now it is heavily referenced from the summary page
# AircraftInfo is the parent widget for all the other aircrafts
class AircraftInfo(QWidget):
    def __init__(self, no_aircraft):
        super(AircraftInfo, self).__init__()
        self.setWindowTitle("Summary Page")
        
        self.waypoint_plaintext_dict = {}
        self.initial_time = 0
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.summary_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
    
        summarised_fields = [
            'Mode', 
            'Status', 
            'Altitude', 
            'Airspeed',  
            'Groundspeed', 
            'GPS', 
            'Throttle', 
            'VTOL Status', 
            'Target Waypoint',
            'Flying Time',
            'Vibe Status',
            'Fuel Level',
            'Quad Battery']

        self.active_aircrafts = no_aircraft
        self.create_summary(self.active_aircrafts, summarised_fields)
        self.statustext_label = QLabel('Status Text')
        self.statustext_label.setContentsMargins(0, 30, 0, 0)
        self.statustext = QPlainTextEdit()
        self.statustext.setReadOnly(True)
        self.statustext.setMinimumHeight(300)

        # Add the widgets into the layouts
        self.main_layout.addLayout(self.summary_layout)
        self.main_layout.addWidget(self.statustext_label)
        self.main_layout.addWidget(self.statustext)

    def create_summary(self, aircraft_no, summarised_fields):
        self.styling()
        self.summary_details_layout = QVBoxLayout()
         # summary_fields_layout will be nested inside summary_details_layout with the progress bar beneath it
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.aircraft_label.setFont(self.h2)
        self.summary_details_layout.addWidget(self.aircraft_label)
        for i in summarised_fields:
            self.summary_fields_layout = QHBoxLayout()
            self.subfield_label_mode = QLabel(i)
            self.subfield_label_mode.setFixedSize(120,30)
            # Create an entry in the dictionary with name aircraftMODE1 and set attributes of the name
            self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setReadOnly(True)

            self.summary_fields_layout.addWidget(self.subfield_label_mode)
            self.summary_fields_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)])
            self.summary_details_layout.addLayout(self.summary_fields_layout)

        self.summary_layout.addLayout(self.summary_details_layout)

    def styling(self):
        self.h2 = QFont("Ubuntu", 15, QFont.Bold) 

    def shutdown(self):
        self.close()

# The child classes as of now are empty -- meaning they do not modify anything from the parent class
# This serves as a placeholder in the case we need to modify anything specific to an aircraft, then we can do it easily
class Aircraft1(AircraftInfo):
    pass
class Aircraft2(AircraftInfo):
    pass
class Aircraft3(AircraftInfo):
    pass
class Aircraft4(AircraftInfo):
    pass
class Aircraft5(AircraftInfo):
    pass
class Aircraft6(AircraftInfo):
    pass
class Aircraft7(AircraftInfo):
    pass
class Aircraft8(AircraftInfo):
    pass
class Aircraft9(AircraftInfo):
    pass
class Aircraft10(AircraftInfo):
    pass
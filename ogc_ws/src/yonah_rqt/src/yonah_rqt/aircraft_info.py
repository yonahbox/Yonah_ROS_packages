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
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPlainTextEdit
from python_qt_binding.QtGui import QFont

# File is still changing rapidly and dynamically, hence comments might not be accurate
# @TODO change the variable names. As of now it is heavily referenced from the summary page
# AircraftInfo is the parent widget for all the other aircrafts
class AircraftInfo(QWidget):
    def __init__(self, aircraft_id):
        super(AircraftInfo, self).__init__()
        self.setWindowTitle("Summary Page")
        
        self.aircraft_info_dict = {}
        self.initial_time = 0
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.info_layout = QVBoxLayout()
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

        self.aircraft_id = aircraft_id
        self.create_info(self.aircraft_id, summarised_fields)
        statustext_label = QLabel('Status Text')
        statustext_label.setContentsMargins(0, 30, 0, 0)
        self.statustext = QPlainTextEdit()
        self.statustext.setReadOnly(True)
        self.statustext.setMinimumHeight(300)

        # Add the widgets into the layouts
        self.main_layout.addLayout(self.info_layout)
        self.main_layout.addWidget(statustext_label)
        self.main_layout.addWidget(self.statustext)

    def create_info(self, aircraft_no, summarised_fields):
        self.styling()
        info_details_layout = QVBoxLayout()
        # info_fields_layout will be nested inside info_details_layout with the progress bar beneath it
        aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        aircraft_label.setFont(self.h2)
        info_details_layout.addWidget(aircraft_label)
        for i in summarised_fields:
            info_fields_layout = QHBoxLayout()
            subfield_label_mode = QLabel(i)
            subfield_label_mode.setFixedSize(120, 30)

            # Create an entry in the dictionary with name aircraftMODE1 and set attributes of the name
            self.aircraft_info_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.aircraft_info_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)
            self.aircraft_info_dict.get('aircraft' + i + str(aircraft_no)).setReadOnly(True)

            info_fields_layout.addWidget(subfield_label_mode)
            info_fields_layout.addWidget(self.aircraft_info_dict['aircraft' + i + str(aircraft_no)])
            info_details_layout.addLayout(info_fields_layout)

        self.info_layout.addLayout(info_details_layout)

    def styling(self):
        self.h2 = QFont("Ubuntu", 15, QFont.Bold) 

    def shutdown(self):
        self.close()

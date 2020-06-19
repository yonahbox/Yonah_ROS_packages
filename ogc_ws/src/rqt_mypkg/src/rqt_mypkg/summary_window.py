#!/usr/bin/env python3
'''
summary_window: Part of the RQt that shows the main information about each aircraft

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
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from checklist_window import ChecklistWindow

### File is still changing rapidly and dynamically, hence comments might not be accurate
class SummaryWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(SummaryWindow, self).__init__()
        self.setWindowTitle("Summary Page")
        self.waypoint_plaintext_dict = {} # Dictionary to store the textedits

        # Create the main_layout
        self.main_layout = QVBoxLayout()
        self.summary_layout = QVBoxLayout()
        self.setLayout(self.main_layout)

        # create the widgets
        summarised_fields = ['Mode', 'Status', 'Airspeed', 'Altitude']
        for i in range(1, active_aircrafts + 1):
            self.create_summary(i, summarised_fields)
        self.statustext = QPlainTextEdit()
        self.statustext.setReadOnly(True)

        # add the widgets into the layouts
        self.main_layout.addWidget(self.statustext)
        self.main_layout.addLayout(self.summary_layout)


    def create_summary(self, aircraft_no, summarised_fields):
        self.summary_details_layout = QVBoxLayout()
         # summary_fields_layout will be nested inside summary_details_layout with the progress bar beneath it
        
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.aircraft_label.setContentsMargins(0, 30, 0, 0)
        self.summary_details_layout.addWidget(self.aircraft_label)
        
        for i in summarised_fields:
            # 
            self.summary_fields_layout = QHBoxLayout()
            self.subfield_label_mode = QLabel(i)
            self.subfield_label_mode.setFixedSize(80,20)

            # Create an entry in the dictionary with name aircraftMODE1 and set attributes of the name
            self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setReadOnly(True)

            self.summary_fields_layout.addWidget(self.subfield_label_mode)
            self.summary_fields_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)])
            self.summary_details_layout.addLayout(self.summary_fields_layout)

        self.summary_layout.addLayout(self.summary_details_layout)
    
    def shutdown(self):
        self.close()
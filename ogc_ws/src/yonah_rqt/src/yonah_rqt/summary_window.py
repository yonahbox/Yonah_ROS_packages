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

from PyQt5.QtWidgets import QVBoxLayout, QPlainTextEdit, QLabel, QHBoxLayout, QWidget
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QFont

class SummaryWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(SummaryWindow, self).__init__()
        self.setWindowTitle("Summary Page")

        # Dictionary to store the textedit names
        self.summary_plaintext_dict = {} 
        # Declare the layouts
        self.main_layout = QVBoxLayout()
        self.summary_layout = QVBoxLayout()
        self.setLayout(self.main_layout) # Set main_layout as the layout that occupies the entire widget
        self.create_layout(active_aircrafts)
        self.statustext = QPlainTextEdit()
        self.statustext.setReadOnly(True)
        # add the widgets into the layouts
        self.main_layout.addWidget(self.statustext)
        self.main_layout.addLayout(self.summary_layout)

    def create_layout(self, active_aircrafts):
        # Declare the widgets
        summarised_fields = ['Mode', 'Status','Airspeed', 'Altitude']
        for i in active_aircrafts:
            self.create_summary(i, summarised_fields)

    def create_summary(self, aircraft_no, summarised_fields):
        self.styling()
        self.summary_details_layout = QVBoxLayout()        
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.aircraft_label.setFont(self.h2)
        self.aircraft_label.setContentsMargins(0, 20, 0, 0)
        self.summary_details_layout.addWidget(self.aircraft_label)
        
        for i in summarised_fields:
            self.summary_fields_layout = QHBoxLayout()
            self.subfield_label_mode = QLabel(i)
            self.subfield_label_mode.setFixedSize(80,20)

            # Create an entry in the dictionary with name aircraftMODE1 and set attributes of the name
            self.summary_plaintext_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.summary_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)
            self.summary_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setReadOnly(True)

            self.summary_fields_layout.addWidget(self.subfield_label_mode)
            self.summary_fields_layout.addWidget(self.summary_plaintext_dict['aircraft' + i + str(aircraft_no)])
            self.summary_details_layout.addLayout(self.summary_fields_layout)

        self.summary_layout.addLayout(self.summary_details_layout)
    
    def styling(self):
        self.h2 = QFont("Ubuntu", 12, QFont.Bold) 

    def open(self):
        self.show()
    
    def shutdown(self):
        self.close()
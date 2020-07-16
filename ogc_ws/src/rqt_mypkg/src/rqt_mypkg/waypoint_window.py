#!/usr/bin/env python3
'''
waypoint_window: Part of the RQt that shows the waypoint status of the aircraft

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
from python_qt_binding.QtWidgets import QWidget, QLabel, QProgressBar, QPlainTextEdit, QVBoxLayout, QHBoxLayout

class WaypointWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(WaypointWindow, self).__init__()
        self.setWindowTitle("Ground Control Station")
        self.adjustSize()
        self.move(200,100)
        
        self.waypoint_plaintext_dict = {}
        
        # Create the layout
        self.main_layout = QHBoxLayout()
        self.buttons_layout = QVBoxLayout()
        self.progressbar_layout = QVBoxLayout()

        # Create the widgets
        for i in range(1, active_aircrafts + 1):
            self.create_progressbar(i)

        # add the widgets into the layouts
        self.main_layout.addLayout(self.progressbar_layout)
        self.setLayout(self.main_layout)

    def create_progressbar(self, aircraft_no):
        waypoint_layout = QVBoxLayout()
        waypoint_layout.setContentsMargins(0,10,0,10)
        waypoint_header_layout = QHBoxLayout() # waypoint_header_layout will be nested inside waypoint_layout with the progress bar beneath it
        
        aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.waypoint_plaintext_dict['aircraft' + str(aircraft_no)] = QPlainTextEdit()
        self.waypoint_plaintext_dict.get('aircraft' + str(aircraft_no)).setMaximumHeight(40)
        self.waypoint_plaintext_dict.get('aircraft' + str(aircraft_no)).setReadOnly(True)
        self.waypoint_plaintext_dict['progress_bar_aircraft' + str(aircraft_no)] = QProgressBar()

        waypoint_header_layout.addWidget(aircraft_label)
        waypoint_header_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + str(aircraft_no)])
        waypoint_layout.addLayout(waypoint_header_layout)
        waypoint_layout.addWidget(self.waypoint_plaintext_dict['progress_bar_aircraft' + str(aircraft_no)])
        
        self.progressbar_layout.addLayout(waypoint_layout)
    
    def shutdown(self):
        self.close()
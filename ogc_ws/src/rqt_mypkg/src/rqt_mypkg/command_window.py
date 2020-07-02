#!/usr/bin/env python3
'''
command_window: Part of the RQt that is responsible to send commands to aircraft

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
from .checklist_window import ChecklistWindow

### File is still changing rapidly and dynamically, hence comments might not be accurate
# Self-note: Consider changing the structure of the code to not over-populate the init
class CommandWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(CommandWindow, self).__init__()
        self.setWindowTitle("Command Window")
        
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.first_row = QHBoxLayout()
        self.second_row = QHBoxLayout()
        # Set main_layout as the layout that occupies the entire widget
        self.setLayout(self.main_layout) 

        # Create the widgets
        self.combo_box = QComboBox()
        # Use a for loop to add items inside the drop down menu
        for i in range(1, active_aircrafts + 1): 
            self.combo_box.addItem('Aircraft ' + str(i))
        self.arm_button = QPushButton('ARM')
        self.disarm_button = QPushButton('DISARM')
        self.go_button = QPushButton('GO / RETURN')
        self.checklist_button = QPushButton('Checklist')
        self.change_identifiers_button = QPushButton('Edit Identifiers')
        self.mission_load_button = QPushButton('Load Mission')
        self.change_mode_button = QPushButton('Change Mode')

        # Set UI properties of the buttons and layout
        top_row = 60 # Minimum height for the top row buttons
        bottom_row = 40 # Minimum height for the bottom row buttons
        self.first_row.setContentsMargins(0,20,0,15)
        self.arm_button.setMinimumHeight(top_row)
        self.disarm_button.setMinimumHeight(top_row)
        self.go_button.setMinimumHeight(top_row)
        self.change_identifiers_button.setMinimumHeight(bottom_row)
        self.mission_load_button.setMinimumHeight(bottom_row)
        self.checklist_button.setMinimumHeight(bottom_row)
        self.change_mode_button.setMinimumHeight(bottom_row)

        # Add the widgets into the layouts
        self.main_layout.addWidget(self.combo_box)
        self.first_row.addWidget(self.arm_button)
        self.first_row.addWidget(self.disarm_button)
        self.first_row.addWidget(self.go_button)
        self.second_row.addWidget(self.checklist_button)
        self.second_row.addWidget(self.change_identifiers_button)
        self.second_row.addWidget(self.mission_load_button)
        self.second_row.addWidget(self.change_mode_button)

        # Add the sub-layouts (first_row and second_row) into the main_layout
        self.main_layout.addLayout(self.first_row)
        self.main_layout.addLayout(self.second_row)

    def shutdown(self):
        self.close()
    

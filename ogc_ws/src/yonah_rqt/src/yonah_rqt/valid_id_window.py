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
import rospy
from std_msgs.msg import UInt8MultiArray
from PyQt5.QtWidgets import QDialog, QDialogButtonBox, QVBoxLayout, QCheckBox
from python_qt_binding.QtCore import Qt

class ValidIdWindow (QDialog):
    def __init__(self):
        super(ValidIdWindow, self).__init__()
        self.setWindowTitle("Input Valid IDs")
        self.max_aircraft = 10
        self.buttons_state = {} # Dictionary to store all the states of the Buttons
        self.pub_valid_ids = rospy.Publisher("ogc/to_telegram/admin/valid_ids", UInt8MultiArray, queue_size = 5)
        self.populate_layout()
        self.setWindowFlags(Qt.WindowStaysOnTopHint)

    
    def populate_layout(self):
        buttons = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(buttons)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.layout = QVBoxLayout()

        for i in range (1, self.max_aircraft+1):
            self.buttons_state[i] = QCheckBox("Aircraft " + str(i))
            self.layout.addWidget(self.buttons_state.get(i))
        
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)

    def accept(self):
        valid_ids = [x for x in self.buttons_state.keys() if self.buttons_state.get(x).isChecked()]
        array_valid_ids = UInt8MultiArray(data=valid_ids)
        self.pub_valid_ids.publish(array_valid_ids)
        rospy.loginfo("Sent valid ids: " + str(valid_ids))
        self.close()

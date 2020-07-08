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
import rospy
from functools import partial

from despatcher.msg import LinkMessage
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from python_qt_binding.QtGui import QFont, QIntValidator
from .checklist_window import ChecklistWindow
from .waypoint_window import WaypointWindow
from .summary_window import SummaryWindow
from .popup_window import *
from .aircraft_info import *
from regular import air_payload

### File is still changing rapidly and dynamically, hence comments might not be accurate
# Self-note: Consider changing the structure of the code to not over-populate the init
class CommandWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(CommandWindow, self).__init__()
        self.setWindowTitle("Command Window")
        
        self.destination_id = 1
        self.send_custom_ping = 0
        self.active_aircrafts = active_aircrafts
        self.checklist_info = {}
        self.arm_status = {}
        for i in range (self.active_aircrafts + 1):
            self.checklist_info["AC" + str(i)] = ChecklistWindow(i)
            
        self.mode_list = ["MANUAL","CIRCLE","STABILIZE","TRAINING","ACRO","FBWA","FBWB","CRUISE","AUTOTUNE","AUTO","RTL",
                          "LOITER","LAND","GUIDED","INITIALISING","QSTABILIZE","QHOVER","QLOITER","QLAND","QRTL"]
        self.decoder = ["MANUAL","CIRCLE","STABILIZE","TRAINING","ACRO","FBWA","FBWB","CRUISE","AUTOTUNE","","AUTO","RTL",
                          "LOITER","","LAND","GUIDED","INITIALISING","QSTABILIZE","QHOVER","QLOITER","QLAND","QRTL"]                  
        self.air = air_payload()
        self.custom_ping_list = self.air.entries.keys()
        self.custom_ping_list.sort()
        self.create_layout()
        
        self.PopupMessages = PopupMessages()
        self.WaypointWindow = WaypointWindow(self.active_aircrafts)
        self.SummaryWindow = SummaryWindow(self.active_aircrafts)

        self.combo_box.currentIndexChanged.connect(self.combo_box_change)
        self.custom_ping_combobox.currentIndexChanged.connect(self.custom_ping_change)

        self.arm_button.pressed.connect(self.arm)
        self.disarm_button.pressed.connect(self.disarm)
        self.go_button.pressed.connect(self.go)
        self.mission_load_button.pressed.connect(self.mission_load)
        self.checklist_button.pressed.connect(self.checklist)
        self.change_mode_button.pressed.connect(self.change_mode)
        self.change_identifiers_button.pressed.connect(self.change_identifiers)
        self.ping_button.pressed.connect(self.ping)
        self.custom_ping_button.pressed.connect(self.custom_ping)

        # Publisher command
        self.command_publisher = rospy.Publisher("ogc/to_despatcher", LinkMessage, queue_size = 5)

    def create_layout(self):
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.first_row = QHBoxLayout()
        self.second_row = QHBoxLayout()
        self.third_row = QHBoxLayout()
        self.ping_row = QFormLayout()
        # Set main_layout as the layout that occupies the entire widget
        self.setLayout(self.main_layout) 

        # Create the widgets
        self.combo_box = QComboBox()
        self.custom_ping_combobox = QComboBox()
        # Use a for loop to add items inside the drop down menu
        for i in range(1, self.active_aircrafts + 1): 
            self.combo_box.addItem('Aircraft ' + str(i))
        for i in (self.custom_ping_list):
            self.custom_ping_combobox.addItem(i)
        self.arm_button = QPushButton('ARM')
        self.disarm_button = QPushButton('DISARM')
        self.go_button = QPushButton('GO / RETURN')
        self.checklist_button = QPushButton('Checklist')
        self.change_identifiers_button = QPushButton('Edit Identifiers')
        self.mission_load_button = QPushButton('Load Mission')
        self.change_mode_button = QPushButton('Change Mode')
        self.ping_button = QPushButton('Ping')
        self.custom_ping_button = QPushButton('Custom Ping Button')
        self.ros_reader = QPushButton('ROS log')

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
        self.ping_button.setMinimumHeight(bottom_row)
        self.custom_ping_button.setMinimumHeight(bottom_row)
        self.custom_ping_combobox.setMinimumHeight(bottom_row)
        self.ros_reader.setMinimumHeight(bottom_row)

        # Add the widgets into the layouts
        self.main_layout.addWidget(self.combo_box)
        self.first_row.addWidget(self.arm_button)
        self.first_row.addWidget(self.disarm_button)
        self.first_row.addWidget(self.go_button)
        self.second_row.addWidget(self.checklist_button)
        self.second_row.addWidget(self.change_identifiers_button)
        self.second_row.addWidget(self.mission_load_button)
        self.second_row.addWidget(self.change_mode_button)
        self.second_row.addWidget(self.ping_button)
        
        self.ping_row.addRow(self.custom_ping_combobox, self.custom_ping_button)

        # Add the sub-layouts (first_row and second_row) into the main_layout
        self.main_layout.addLayout(self.first_row)
        self.main_layout.addLayout(self.second_row)
        self.third_row.addLayout(self.ping_row)
        self.third_row.addWidget(self.ros_reader)
        self.main_layout.addLayout(self.third_row)

    def create_link_message(self, destination_id, data):
        message = LinkMessage()
        message.id = destination_id
        message.data = data
        self.command_publisher.publish(message)

    def combo_box_change(self, i):
        self.destination_id = i + 1

    def custom_ping_change(self, i):
        self.send_custom_ping = i

    def custom_ping(self):
        data = "ping " + str(self.custom_ping_list[self.send_custom_ping])
        self.create_link_message(self.destination_id, data)

    def arm (self):
        if self.checklist_info.get("AC" + str(self.destination_id)).checklist_state == 0:
            self.PopupMessages.arm_window(self.destination_id, ["ARM","Warning"], "Warning Message", "You have not completed pre-flight checklist", "Are you sure you want to ARM?")
        else:
            self.PopupMessages.arm_window(self.destination_id, ["ARM", "Information"], "Confirmation Message", "Please confirm your action", "Are you sure you want to ARM?")
        if self.arm_status.get('AC' + str(self.destination_id)) == "DISARMED":
            pass
            # self.aircrafts_info.get("AC" + str(self.destination_id)).initial_time = self.time
            # rospy.logdebug("[AIRCRAFT ARM LIST] %s", self.aircrafts_info)
            # print('time start')

    def disarm (self):
        self.PopupMessages.arm_window(self.destination_id, ["DISARM", "Information"], "Confirmation Message", "Please confirm your action", "Are you sure you want to DISARM?")

    def go(self):
        data = "mode 10"
        statustext_message = "Aircraft {} mode has been set to AUTO".format(self.destination_id)
        self.create_link_message(self.destination_id, data)
        rospy.logdebug("[AC %d go_button] %s", self.destination_id, statustext_message)

    def mission_load(self):
        self.PopupMessages.user_input_textbox("Waypoint Load", "Load waypoint to Aircraft ", self.destination_id)
        if self.PopupMessages.input_text == []:
            return 0 # When else is clicked, return nothing
        elif self.PopupMessages.input_text[0] == "":
            statustext_message = "ERROR: Please input a valid waypoint file"
        else:
            data = self.PopupMessages.input_text[0]
            load_destination_id = self.PopupMessages.input_text[1]
            statustext_message = "Waypoint file: {} uploaded to Aircraft {}".format(data, load_destination_id)
            self.create_link_message(load_destination_id, data)
            rospy.logdebug("[AC %d Mission Load Button] %s", self.destination_id, statustext_message)

        self.SummaryWindow.statustext.appendPlainText(statustext_message)

    def ping(self):
        data = 'ping'
        self.create_link_message(self.destination_id, data)

    def checklist(self):
        self.checklist_info.get("AC" + str(self.destination_id)).show()

    def change_identifiers(self):
        self.change_identifiers_dialog = QDialog()
        self.change_identifiers_dialog.setWindowTitle("Edit Identifiers")
        layout = QVBoxLayout(self.change_identifiers_dialog)
        add_identifiers_button = QPushButton("Add New Identifiers")
        edit_identifiers_button = QPushButton("Edit Existing Identifiers")
        add_identifiers_button.setMinimumHeight(40)
        edit_identifiers_button.setMinimumHeight(40)
        add_identifiers_button.pressed.connect(self.add_identifiers)
        edit_identifiers_button.pressed.connect(self.edit_identifiers)
        layout.addWidget(add_identifiers_button)
        layout.addWidget(edit_identifiers_button)
        self.change_identifiers_dialog.show()

    def add_identifiers(self):
        self.identifiers_dialog = QDialog()
        self.identifiers_dialog.setWindowTitle("Add New Identifiers")
        title = QLabel("Add New Identifiers")
        title.setFont(QFont("Ubuntu", 13, QFont.Bold))
        title.setContentsMargins(0, 0, 0, 10)

        identifiers_layout = QFormLayout()
        identifiers_layout.addRow(title)
    
        name = QLabel("Label")
        name_lineedit = QLineEdit()
        identifiers_layout.addRow(name, name_lineedit)

        phone = QLabel("Phone number")
        phone_lineedit = QLineEdit()
        phone_lineedit.setValidator(QIntValidator())
        phone_lineedit.setMaxLength(10)
        identifiers_layout.addRow(phone, phone_lineedit)

        imei = QLabel("IMEI number")
        imei_lineedit = QLineEdit()
        imei_lineedit.setValidator(QIntValidator())
        identifiers_layout.addRow(imei, imei_lineedit)

        serial = QLabel("Serial number")
        serial_lineedit = QLineEdit()
        serial_lineedit.setValidator(QIntValidator())
        identifiers_layout.addRow(serial, serial_lineedit)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel,
            centerButtons=True,)
        buttons.accepted.connect(self.identifiers_accept)
        buttons.rejected.connect(self.identifiers_dialog.close)

        identifiers_layout.addWidget(buttons)
        self.identifiers_dialog.setLayout(identifiers_layout)
        self.identifiers_dialog.show()

    def identifiers_accept(self):
        ### Do something after Rumesh's identifiers is settled
        self.identifiers_dialog.close()
        self.change_identifiers_dialog.close()

    def edit_identifiers(self):
        self.edit_identifiers_dialog = QDialog()
        self.edit_identifiers_dialog.setWindowTitle("Edit Identifiers")

        layout = QVBoxLayout(self.edit_identifiers_dialog)
        ground_button = QPushButton("Ground Identifier")
        ground_button.setMinimumHeight(40)
        ground_button.pressed.connect(partial(self.ground_air_identifiers, "Ground"))

        air_button = QPushButton("Air Identifier")
        air_button.setMinimumHeight(40)
        air_button.pressed.connect(partial(self.ground_air_identifiers, "Air"))

        back_button = QPushButton("Cancel")
        back_button.setMaximumSize(80, 30)
        back_button.pressed.connect(self.edit_identifiers_dialog.close)

        layout.addWidget(ground_button)
        layout.addWidget(air_button)
        layout.addWidget(back_button)
        self.edit_identifiers_dialog.show()
    
    def ground_air_identifiers(self, mode):
        self.change_identifiers_dialog.close()
        self.edit_ground_air_dialog = QDialog()
        self.edit_ground_air_dialog.setWindowTitle("Edit {} Identifier".format(mode))

        title = QLabel("Add New Identifiers")
        title.setFont(QFont("Ubuntu", 13, QFont.Bold))
        title.setContentsMargins(0, 0, 0, 10)

        label = QLabel("Select Aircraft label to edit")
        combo_box = QComboBox()
        # Somehow get the current label inside it
        for i in range (1, self.active_aircrafts + 1):
            combo_box.addItem(str(i))
        combo_box.currentIndexChanged.connect(self.edit_identifiers_combo_box)

        name = QLabel("Label")
        name_lineedit = QLineEdit()

        phone = QLabel("Phone number")
        phone_lineedit = QLineEdit()
        phone_lineedit.setValidator(QIntValidator())
        phone_lineedit.setMaxLength(10)

        imei = QLabel("IMEI number")
        imei_lineedit = QLineEdit()
        imei_lineedit.setValidator(QIntValidator())

        serial = QLabel("Serial number")
        serial_lineedit = QLineEdit()
        serial_lineedit.setValidator(QIntValidator())

        box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel,
            centerButtons=True,)

        if mode == "Ground":
            box.accepted.connect(self.edit_ground_accept)
        else:
            box.accepted.connect(self.edit_air_accept)
        box.rejected.connect(self.edit_ground_air_dialog.close)

        lay = QFormLayout(self.edit_ground_air_dialog)
        lay.addRow(title)
        lay.addRow(label, combo_box)
        lay.addRow(name, name_lineedit)
        lay.addRow(phone, phone_lineedit)
        lay.addRow(imei, imei_lineedit)
        lay.addRow(serial, serial_lineedit)

        lay.addWidget(box)
        self.edit_ground_air_dialog.show()

    def edit_ground_accept(self):
        self.edit_ground_air_dialog.close()
        

    def edit_air_accept(self):
        self.edit_ground_air_dialog.close()
        
    
    def edit_identifiers_combo_box(self, i):
        self.edit_identifiers_id = i + 1

    def change_mode(self):
        self.change_mode_dialog = QDialog()
        self.change_mode_dialog.setWindowTitle("Change Mode")
        self.label = QLabel("Select MODE for Aircraft " + str(self.destination_id))
        self.combo_box = QComboBox()
        for i in self.mode_list:
            self.combo_box.addItem(i)
        self.combo_box.currentIndexChanged.connect(self.mode_combo_box)
        self.mode_change = self.combo_box.currentText()
        box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel,
            centerButtons=True,)
        box.accepted.connect(self.dropdown_accept)
        box.rejected.connect(self.change_mode_dialog.close)
        lay = QVBoxLayout(self.change_mode_dialog)
        lay.addWidget(self.label)
        lay.addWidget(self.combo_box)
        lay.addWidget(box)
        self.change_mode_dialog.show()

    def mode_combo_box(self, i):
        self.mode_change = self.mode_list[i]

    def dropdown_accept(self):
        data = "mode " + str(self.decoder.index(self.mode_change))
        statustext_message = "Aircraft {} mode has been set to {}".format(self.destination_id, self.mode_change)
        self.SummaryWindow.statustext.appendPlainText(statustext_message)
        self.create_link_message(self.destination_id, data)
        self.change_mode_dialog.close()
        rospy.logdebug("[AC %d Change Mode] %s", self.destination_id, statustext_message)
    def shutdown(self):
        self.close()
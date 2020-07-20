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

from functools import partial
from PyQt5.QtWidgets import QWidget, QShortcut, QVBoxLayout, QHBoxLayout, QFormLayout, QComboBox
from PyQt5.QtWidgets import QScrollArea, QFrame, QPushButton, QDialog, QLabel, QLineEdit, QDialogButtonBox
from python_qt_binding.QtCore import Qt, Signal, Slot, QRegExp
from python_qt_binding.QtGui import QFont, QRegExpValidator, QValidator

from regular import air_payload
from despatcher.msg import LinkMessage
from identifiers.srv import AddNewDevice, AddNewDeviceRequest, GetIds
from identifiers.srv import EditDevice, EditDeviceRequest
from identifiers.srv import GetAllDetails, GetAllDetailsRequest
from identifiers.srv import GetDetails, GetSelfDetails
from .checklist_window import ChecklistWindow
from .waypoint_window import WaypointWindow
from .summary_window import SummaryWindow
from .popup_window import PopupMessages

class CommandWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(CommandWindow, self).__init__()
        self.setWindowTitle("Command Window")
        
        self.destination_id = 1
        self.edit_identifiers_id = 1
        self.identifiers_error = 0
        self.send_custom_ping = 0
        self.is_window_open = [0,0,0] # Keeps record whether change identifier, add identifier, edit identifier windows are open
        self.active_aircrafts = active_aircrafts
        self.checklist_info = {}
        self.arm_status = {}

        
        
        # Define the mode list as well as the decoder
        self.mode_list = ["MANUAL","CIRCLE","STABILIZE","TRAINING","ACRO","FBWA","FBWB","CRUISE","AUTOTUNE","AUTO","RTL",
                          "LOITER","LAND","GUIDED","INITIALISING","QSTABILIZE","QHOVER","QLOITER","QLAND","QRTL"]
        self.decoder = ["MANUAL","CIRCLE","STABILIZE","TRAINING","ACRO","FBWA","FBWB","CRUISE","AUTOTUNE","","AUTO","RTL",
                          "LOITER","","LAND","GUIDED","INITIALISING","QSTABILIZE","QHOVER","QLOITER","QLAND","QRTL"]                  

        # Get the headers for the payload
        self.air = air_payload()
        self.custom_ping_list = self.air.entries.keys()
        self.custom_ping_list.sort()

        self.create_layout(active_aircrafts)

        self.PopupMessages = PopupMessages()
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

        # Publisher Command
        self.command_publisher = rospy.Publisher("ogc/to_despatcher", LinkMessage, queue_size = 5)

        # Service Command
        # try:
        #     rospy.wait_for_service("identifiers/get/imei", timeout= 5)
        #     rospy.wait_for_service("identifiers/get/serial", timeout= 5)
        #     rospy.wait_for_service("identifiers/get/all", timeout= 5)
        #     rospy.wait_for_service("identifiers/get/number", timeout= 5)

        #     rospy.wait_for_service("identifiers/add/device", timeout= 5)
        #     rospy.wait_for_service("identifiers/edit/device", timeout= 5)
        #     rospy.wait_for_service("identifiers/get/ids", timeout= 5)

        # except rospy.ROSException:
        #     self.PopupMessages.warning_message("Identifiers Node failed to start", "Edit and Add Identifiers function will be unavailable")
        #     self.identifiers_error = 1
        #     rospy.logerr("Identifiers node is not initialised")
            
        self.add_new_device = rospy.ServiceProxy("identifiers/add/device", AddNewDevice)
        self.edit_device = rospy.ServiceProxy("identifiers/edit/device", EditDevice)
        self.get_device = rospy.ServiceProxy("identifiers/get/all", GetAllDetails)
        self.get_number = rospy.ServiceProxy("identifiers/get/number", GetDetails)
        self.get_imei = rospy.ServiceProxy("identifiers/get/imei", GetDetails)
        self.get_serial = rospy.ServiceProxy("identifiers/get/serial", GetDetails)
        self.get_ids = rospy.ServiceProxy("identifiers/get/ids", GetIds)

    def create_layout(self, active_aircrafts):
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.secondary_layout = QVBoxLayout()
        self.full_layout = QVBoxLayout()
        self.first_row = QHBoxLayout()
        self.second_row = QHBoxLayout()
        self.third_row = QHBoxLayout()
        self.ping_row = QFormLayout()

        # Set main_layout as the layout that occupies the entire widget
        self.setLayout(self.main_layout) 

        # Create the widgets
        self.combo_box = QComboBox()
        self.custom_ping_combobox = QComboBox()

        self.create_combobox(active_aircrafts)
        # Use a for loop to add items inside the drop down menu
        
        for i in (self.custom_ping_list):
            self.custom_ping_combobox.addItem(i)

        self.scroll_area = QScrollArea()
        self.frame1 = QFrame()
        self.arm_button = QPushButton('ARM')
        self.disarm_button = QPushButton('DISARM')
        self.go_button = QPushButton('GO / RETURN')
        self.checklist_button = QPushButton('Checklist')
        self.change_identifiers_button = QPushButton('Change Identifiers')
        self.mission_load_button = QPushButton('Load Mission')
        self.change_mode_button = QPushButton('Change Mode')
        self.ping_button = QPushButton('Ping')
        self.custom_ping_button = QPushButton('Custom Ping Button')
        self.ros_reader = QPushButton('ROS log')

        # Set UI properties of the buttons and layout
        top_row = 60        # Minimum height for the top row buttons
        bottom_row = 40     # Minimum height for the bottom row buttons
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
        self.secondary_layout.addWidget(self.combo_box)
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
        self.secondary_layout.addLayout(self.first_row)
        self.secondary_layout.addLayout(self.second_row)
        self.third_row.addLayout(self.ping_row)
        self.third_row.addWidget(self.ros_reader)
        self.secondary_layout.addLayout(self.third_row)
        self.frame1.setLayout(self.secondary_layout)
        self.scroll_area.setWidget(self.frame1)
        self.scroll_area.setMaximumHeight(500)
        self.scroll_area.setMinimumWidth(600)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.main_layout.addWidget(self.scroll_area)

    def create_combobox(self, active_aircrafts):
        for i in range(self.combo_box.count(),-1,-1):
            self.combo_box.removeItem(i)
        for j in active_aircrafts: 
            self.combo_box.addItem('Aircraft ' + str(j))
            self.checklist_info["AC" + str(j)] = ChecklistWindow(j) # Create the checklist as well

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

    def arm(self):
        if self.checklist_info.get("AC" + str(self.destination_id)).checklist_state == 0:
            self.PopupMessages.arm_window(self.destination_id, ["ARM","Warning"], "Warning Message", "You have not completed pre-flight checklist", "Are you sure you want to ARM?")
        else:
            self.PopupMessages.arm_window(self.destination_id, ["ARM", "Information"], "Confirmation Message", "Please confirm your action", "Are you sure you want to ARM?")
        if self.arm_status.get('AC' + str(self.destination_id)) == "DISARMED":
            pass

    def disarm(self):
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
        if self.identifiers_error == 1:
            rospy.logerr("Attempt to open change identifiers window without initialising identifiers node")
            self.PopupMessages.warning_message("Identifiers Node failed to start", "Edit and Add Identifiers function will be unavailable")
            return 0
        
        self.change_identifiers_dialog = QDialog()
        self.change_identifiers_dialog.setWindowTitle("Edit Identifiers")
        layout = QVBoxLayout(self.change_identifiers_dialog)

        add_ground = QPushButton("Add New Ground Identifiers")
        add_air = QPushButton("Add New Air Identifiers")
        edit_ground = QPushButton("Edit Existing Ground Identifiers")
        edit_air = QPushButton("Edit Existing Air Identifiers")

        add_ground.setMinimumHeight(40)
        add_air.setMinimumHeight(40)
        edit_ground.setMinimumHeight(40)
        edit_air.setMinimumHeight(40)

        add_ground.pressed.connect(partial(self.add_identifiers, "Ground"))
        add_air.pressed.connect(partial(self.add_identifiers, "Air"))
        edit_ground.pressed.connect(partial(self.edit_identifiers, "Ground"))
        edit_air.pressed.connect(partial(self.edit_identifiers, "Air"))

        layout.addWidget(add_ground)
        layout.addWidget(add_air)
        layout.addWidget(edit_ground)
        layout.addWidget(edit_air)

        self.change_identifiers_dialog.show()
        self.is_window_open[0] = 1

    def add_identifiers(self, side):
        self.side = side
        self.change_identifiers_dialog.close()
        self.is_window_open[0] = 0

        self.add_identifiers_dialog = QDialog()
        self.add_identifiers_dialog.setWindowTitle("Add {} Identifier".format(side))
        self.is_window_open[1] = 1

        title = QLabel("Add New {} Identifier".format(side))
        title.setFont(QFont("Ubuntu", 13, QFont.Bold))
        title.setContentsMargins(0, 0, 0, 10)

        identifiers_layout = QFormLayout()
        identifiers_layout.addRow(title)
        self.check_validity = [0, 0, 0, 0]

        self.identifiers_input_field("add")
        identifiers_layout.addRow(self.name, self.name_lineedit)
        identifiers_layout.addRow(self.phone, self.phone_lineedit)
        identifiers_layout.addRow(self.imei, self.imei_lineedit)
        identifiers_layout.addRow(self.serial, self.serial_lineedit)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel,
            centerButtons=True,)
        buttons.accepted.connect(partial(self.add_identifiers_accept, side))
        buttons.rejected.connect(partial(self.close_identifiers, "Add"))

        identifiers_layout.addWidget(buttons)
        self.add_identifiers_dialog.setLayout(identifiers_layout)
        self.add_identifiers_dialog.show()

    def identifiers_input_field(self, field):
        self.name = QLabel("Label")
        self.name_lineedit = QLineEdit()
        self.name_lineedit.setValidator(QRegExpValidator(QRegExp("^[a-zA-Z0-9_]{2,15}$")))
        
        self.phone = QLabel("Phone number")
        self.phone_lineedit = QLineEdit()
        self.phone_lineedit.setValidator(QRegExpValidator(QRegExp("[0-9.-]{10,12}$")))
        
        self.imei = QLabel("IMEI number")
        self.imei_lineedit = QLineEdit()
        self.imei_lineedit.setValidator(QRegExpValidator(QRegExp("[0-9]\\d{14}")))
        
        self.serial = QLabel("Serial number")
        self.serial_lineedit = QLineEdit()
        self.serial_lineedit.setValidator(QRegExpValidator(QRegExp("[0-9]\\d{4}")))
        
        self.name_lineedit.textChanged.connect(partial(self.identifiers_submit, field, "label"))
        self.phone_lineedit.textChanged.connect(partial(self.identifiers_submit, field, "phone"))
        self.imei_lineedit.textChanged.connect(partial(self.identifiers_submit, field, "imei"))
        self.serial_lineedit.textChanged.connect(partial(self.identifiers_submit, field, "serial"))

        # Additional Information upon hover of textboxes
        label_msg = "Only 2-15 digit alpha-numeric character and _ is allowed \n E.g. Yonah_AC1"
        phone_msg = "Only 10-12 digit numeric character is allowed \n Please include the country code E.g. 6593511282"
        imei_msg = "Only 15 digit numeric character is allowed \n E.g. 251902125903967"
        serial_msg = "Only 5 digit numeric character is allowed \n E.g. 12345"

        self.name.setToolTip(label_msg)
        self.name_lineedit.setToolTip(label_msg)
        self.phone.setToolTip(phone_msg)
        self.phone_lineedit.setToolTip(phone_msg)
        self.imei.setToolTip(imei_msg)
        self.imei_lineedit.setToolTip(imei_msg)
        self.serial.setToolTip(serial_msg)
        self.serial_lineedit.setToolTip(serial_msg)

    def identifiers_submit(self, field, subfields, text):
        sender = self.sender()
        validator = sender.validator()
        state = validator.validate(text, 0)[0]
        if state == QValidator.Acceptable:
            color = '#c4df9b' # green
        elif state == QValidator.Intermediate:
            color = '#fff79a' # yellow
        else:
            color = '#f6989d' # red
        sender.setStyleSheet('QLineEdit { background-color: %s }' % color)

        if subfields == "label":
            if field == "add":
                self.add_label = text
            else:
                self.edit_label = text
            if state == QValidator.Acceptable:
                self.check_validity[0] = 1  
            else: 
                self.check_validity[0] = 2
        elif subfields == "phone":
            if field == "add":
                self.add_phone = text
            else:
                self.edit_phone = text
            if state == QValidator.Acceptable:
                self.check_validity[1] = 1  
            else: 
                self.check_validity[1] = 2
        elif subfields == "imei":
            if field == "add":
                self.add_imei = text
            else:
                self.edit_imei = text
            if state == QValidator.Acceptable:
                self.check_validity[2] = 1  
            else: 
                self.check_validity[2] = 2
        elif subfields == "serial":
            if field == "add":
                self.add_serial = text
            else:
                self.edit_serial = text
            if state == QValidator.Acceptable:
                self.check_validity[3] = 1  
            else: 
                self.check_validity[3] = 2

    def add_identifiers_accept(self, side):
        if sum(self.check_validity) != 4:
            self.PopupMessages.warning_message("Some input fields requirement have not been met.\nPlease check your inputs again", "Hover mouse over input field for more information")
            return 0
        new_identifier = AddNewDeviceRequest()
        new_identifier.label = self.add_label
        new_identifier.number = self.add_phone
        new_identifier.imei = self.add_imei
        new_identifier.rb_serial = self.add_serial
        if side == "Air":
            new_identifier.is_air = True
        else:
            new_identifier.is_air = False
        add_identifier = self.add_new_device(new_identifier)
        rospy.loginfo(add_identifier.success)
        self.add_identifiers_dialog.close()
        self.is_window_open[1] = 0

    def edit_identifiers(self, side):
        self.side = side
        self.change_identifiers_dialog.close()
        self.is_window_open[0] = 0

        ids_response = self.get_ids()
        self.air_ids = ids_response.air_ids
        self.ground_ids = ids_response.ground_ids

        self.edit_identifiers_dialog = QDialog()
        self.is_window_open[2] = 1
        self.edit_identifiers_dialog.setWindowTitle("Edit {} Identifier".format(side))
        title = QLabel("Edit Existing {} Identifier".format(side))
        title.setFont(QFont("Ubuntu", 13, QFont.Bold))
        title.setContentsMargins(0, 0, 0, 10)

        label = QLabel("Select Aircraft label to edit")
        self.edit_combo_box = QComboBox()
        self.edit_identifiers_id = 1
        # Somehow get the current label inside it
        if side == "Air":
            for i in self.air_ids:
                self.edit_combo_box.addItem("Aircraft " + chr(ord(i) + 48))
        else:
            for i in self.ground_ids:
                self.edit_combo_box.addItem("GCS " + chr(ord(i) + 48))
        self.edit_combo_box.currentIndexChanged.connect(self.edit_identifiers_combo_box)

        self.check_validity = [0, 0, 0, 0]
        self.identifiers_input_field("edit")

        box = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel,
            centerButtons=True,)
        box.accepted.connect(partial(self.edit_identifiers_accept, side))
        box.rejected.connect(partial(self.close_identifiers, "Edit"))

        lay = QFormLayout(self.edit_identifiers_dialog)
        lay.addRow(title)
        lay.addRow(label, self.edit_combo_box)
        lay.addRow(self.name, self.name_lineedit)
        lay.addRow(self.phone, self.phone_lineedit)
        lay.addRow(self.imei, self.imei_lineedit)
        lay.addRow(self.serial, self.serial_lineedit)

        self.changed()
        lay.addWidget(box)
        self.edit_identifiers_dialog.show()

    def close_identifiers(self, side):
        if side == "Edit":
            self.edit_identifiers_dialog.close()
            self.is_window_open[2] = 0
        else:
            self.add_identifiers_dialog.close()
            self.is_window_open[1] = 0
        self.change_identifiers_dialog.show()
        self.is_window_open[0] = 1

    def edit_identifiers_accept(self, side):
        if sum(self.check_validity) != 4:
            self.PopupMessages.warning_message("Some input fields requirement have not been met.\nPlease check your inputs again", "Hover mouse over input field for more information")
            return 0

        edit_identifiers = EditDeviceRequest()
        edit_identifiers.id = self.edit_identifiers_id
        rospy.loginfo("ID that is sent: " + str(edit_identifiers.id))
        edit_identifiers.label = self.edit_label
        edit_identifiers.number = self.edit_phone
        edit_identifiers.imei = self.edit_imei
        edit_identifiers.rb_serial = self.edit_serial
        if side == "Air":
            edit_identifiers.is_air = True
        else:
            edit_identifiers.is_air = False
        edit_result = self.edit_device(edit_identifiers)
        rospy.loginfo("Result: " + str(edit_result.success))
        self.edit_identifiers_dialog.close()
        self.is_window_open[2] = 0
    
    def edit_identifiers_combo_box(self, i):
        self.edit_identifiers_id = i + 1 # Identifiers start with index 1
        self.changed()

    def changed(self):
        current_identifier = GetAllDetailsRequest()
        current_identifier.id = self.edit_identifiers_id
        current_identifier.is_air = True if self.side == "Air" else False
        try:
            new_edit_device = self.get_device(current_identifier)
            self.name_lineedit.setText(str(new_edit_device.label))
            self.edit_label = new_edit_device.label
            self.phone_lineedit.setText(str(new_edit_device.number))
            self.edit_phone = new_edit_device.number
            self.imei_lineedit.setText(str(new_edit_device.imei))
            self.edit_imei = new_edit_device.imei
            self.serial_lineedit.setText(str(new_edit_device.rb_serial))
            self.edit_serial = new_edit_device.rb_serial
        except rospy.ServiceException as e:
            self.edit_combo_box.setCurrentIndex(0)
            self.PopupMessages.warning_message("The ID you have selected is invalid", "Please select another ID")

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
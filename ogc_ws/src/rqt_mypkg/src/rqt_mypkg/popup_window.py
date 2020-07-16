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
import rospy
import threading
from despatcher.msg import LinkMessage
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import Qt
from despatcher.msg import LinkMessage

class PopupMessages(QWidget):
    def __init__(self):
        super(PopupMessages, self).__init__()
        self.setWindowTitle("Command Window")
        self.move(700,400)
        self.command_publisher = rospy.Publisher("ogc/to_despatcher", LinkMessage, queue_size = 5)

    def create_link_message(self, destination_id, data):
        '''Create a custom Link Message'''
        message = LinkMessage()
        message.id = destination_id
        message.data = data
        self.command_publisher.publish(message)

    def user_input_textbox(self, title, message, id):
        text, ok = QInputDialog.getText(self, title, message + str(id))
        if ok:
            self.input_text = [text, id]
        else:
            self.input_text = []
    
    def emergency_disarm(self):
        num,ok = QInputDialog.getInt(self,"Emergency Disarm","Enter Aircraft Number for EMERGENCY DISARM")
        if ok:
            data = "disarm"
            if num == 0:
                rospy.logerr("Invalid Aircraft ID, Please input a valid Aircraft ID")
                return 0
            self.create_link_message(num, data)
            rospy.logdebug("[AC %d EMERGENCY DISARM]", num)
    
    def arm_window(self, id, message_type, title, message, text = "Do you still want to continue?"):
        self.destination_id = id
        self.message = QMessageBox()
        self.has_message_opened = 1
        if message_type[1] == "Warning":
            self.message.setIcon(QMessageBox.Warning)
        elif message_type[1] == "Information":
            self.message.setIcon(QMessageBox.Information)
        self.message.setText(message)
        self.message.setInformativeText(text)
        self.message.setWindowTitle(title)
        self.message.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        self.message.show()
        if message_type[0] == "ARM":
            self.message.buttonClicked.connect(self.arm_message)
        elif message_type[0] == "DISARM":
            self.message.buttonClicked.connect(self.disarm_message)

    def arm_message(self, i):
        if i.text() == '&Yes':
            data = "arm"
            statustext_message = "Aircraft {} ARM command sent".format(self.destination_id)
            self.create_link_message(self.destination_id, data)
            rospy.logdebug("[AC %d arm_button] %s", self.destination_id, statustext_message)
        else:
            self.message.close()

    def disarm_message(self, i):
        if i.text() == '&Yes':
            data = "disarm"
            statustext_message = "Aircraft {} DISARM command sent".format(self.destination_id)
            self.create_link_message(self.destination_id, data)
            rospy.logdebug("[AC %d disarm_button] %s", self.destination_id, statustext_message)
        else:
            self.message.close()
    
    def warning_message(self, heading, text):
        self.warning = QMessageBox()
        self.warning.setIcon(QMessageBox.Warning)
        self.warning.setText(heading)
        self.warning.setInformativeText(text)
        self.warning.setWindowTitle("Error Message")
        self.warning.setStandardButtons(QMessageBox.Ok)
        self.warning.buttonClicked.connect(self.warning.close)
        self.warning.show()
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
    
    def confirmation_window(self, title, message, text = "Do you still want to continue?"):
        self.message = QMessageBox()
        self.has_message_opened = 1
        self.message.setIcon(QMessageBox.Warning)
        self.message.setText(message)
        self.message.setInformativeText(text)
        self.message.setWindowTitle(title)
        self.message.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        self.message.show()
        self.message.buttonClicked.connect(self.message_action)

    # Determines what happens after dialog_window pops up from ok_button
    def message_action(self, i):
        if i.text() == '&Yes':
            self.response = 'yes'
            # __main__.my_module.MyPlugin.status_text_display(__main__.my_module.MyPlugin, 'Checklist has been uploaded')
        else:
            self.response = 'no'
            self.message.close()
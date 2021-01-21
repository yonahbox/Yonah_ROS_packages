#!/usr/bin/env python3
'''
checklist_window: Additional window that contains the BPO and BTO checklists

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
import csv
import os
import rospkg
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QColor

class LogWindow(QWidget):
    def __init__(self, file, path):
        super(LogWindow, self).__init__()
        # Set properties of the window
        self.setWindowTitle("ROS Log Reader")
        self.resize(1010, 700)
        self.setMinimumWidth(1010)
        self.move(200,100)

        self.file = file
        ct = 0
        for i in range (len(path)-1, 0, -1):
            if path[i] == "/":
                ct += 1
            if ct == 3:
                path = "... " + path[i:]
                break
        self.path = path
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.buttons_layout = QHBoxLayout()

        # Create the widgets
        self.create_layout()
        self.condensed_layout()

        # Connect the Buttons
        self.condense.pressed.connect(self.condensed_layout)
        self.long.pressed.connect(self.lengthy_layout)
        self.path_button.pressed.connect(self.reload_file)
        # Add the widgets into the layouts
        self.setLayout(self.main_layout)
        self.show()

    # Create the main layout of widget
    def create_layout(self):
        path_label = QLabel("Path: ")
        self.path_button = QPushButton(self.path)
        self.condense = QPushButton("Condensed Version")
        self.long = QPushButton("Full Version")
        self.buttons_layout.addWidget(path_label)
        self.buttons_layout.addWidget(self.path_button)
        self.buttons_layout.addWidget(self.condense)
        self.buttons_layout.addWidget(self.long)
        self.buttons_layout.setAlignment(Qt.AlignLeft)

        self.table_widget = QTableWidget()
        self.table_widget.setSizeAdjustPolicy(QAbstractScrollArea.AdjustToContents)
        self.table_widget.setMinimumWidth(1000)
        self.main_layout.addLayout(self.buttons_layout)
        self.main_layout.addWidget(self.table_widget)

    def condensed_layout(self):
        '''Condense the information in the table if it is too long'''
        self.table_widget.setRowCount(0)
        self.file = [x for x in self.file if x]
        self.table_widget.setRowCount(len(self.file))
        self.table_widget.setColumnCount(4)
        j = 0
        for i in self.file:
            if not i:
                continue
            i = i.replace(":", ";", 2)
            i = i.replace("[", ":[", 2)
            i = i.replace("]", "]:", 2)
            i = i.split(':', 5)
            i = list(filter(None, i))
            if len(i) != 4:
                self.table_widget.setItem(j, 0, QTableWidgetItem(""))
                self.table_widget.setItem(j, 1, QTableWidgetItem(""))
                self.table_widget.setItem(j, 2, QTableWidgetItem(""))
                self.table_widget.setItem(j, 3, QTableWidgetItem(" ".join(i)))
            else:
                ros_type = QTableWidgetItem(i[0])
                item = QTableWidgetItem(i[1])
                if i[1] == "[ERROR]":
                    item.setForeground(QColor(255, 0, 0))
                    ros_type.setForeground(QColor(255, 0, 0))
                elif i[1] == "[WARNING]":
                    item.setForeground(QColor(218,165,32))
                    ros_type.setForeground(QColor(218,165,32))
                self.table_widget.setItem(j, 0, QTableWidgetItem(ros_type))
                self.table_widget.setItem(j, 1, item)
                self.table_widget.setItem(j, 2, QTableWidgetItem(i[2].replace(";", ":")))
                self.table_widget.setItem(j, 3, QTableWidgetItem(i[3][:500]))
                if len(i[3]) > 499:
                    self.table_widget.setItem(j, 3, QTableWidgetItem(i[3][:500] + "..."))         
            j += 1

        self.table_widget.setHorizontalHeaderLabels(["Message Type","Priority", "Time", "Message"])
        header = self.table_widget.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        self.table_widget.resizeRowsToContents()

    def lengthy_layout(self):
        '''Show the full length of message in the table'''
        self.table_widget.setRowCount(0)
        self.file = [x for x in self.file if x]
        self.table_widget.setRowCount(len(self.file))
        self.table_widget.setColumnCount(4)
        j = 0
        for i in self.file:
            if not i:
                continue
            i = i.replace(":", ";", 2)
            i = i.replace("[", ":[", 2)
            i = i.replace("]", "]:", 2)
            i = i.split(':', 5)
            i = list(filter(None, i))
            if len(i) != 4:
                self.table_widget.setItem(j, 0, QTableWidgetItem(""))
                self.table_widget.setItem(j, 1, QTableWidgetItem(""))
                self.table_widget.setItem(j, 2, QTableWidgetItem(""))
                self.table_widget.setItem(j, 3, QTableWidgetItem(" ".join(i)))
            else:
                self.table_widget.setItem(j, 0, QTableWidgetItem(i[0]))
                item = QTableWidgetItem(i[1])
                if item == "[ERROR]":
                    item.setForeground(QColor(255, 0, 0))
                self.table_widget.setItem(j, 1, item)
                self.table_widget.setItem(j, 2, QTableWidgetItem(i[2].replace(";", ":")))
                self.table_widget.setItem(j, 3, QTableWidgetItem(i[3][:5000]))
                if len(i[3]) > 499:
                    self.table_widget.setItem(j, 3, QTableWidgetItem(i[3][:5000] + "..."))         
            j += 1
        self.table_widget.setHorizontalHeaderLabels(["Message Type","Priority", "Time", "Message"])
        header = self.table_widget.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        self.table_widget.resizeRowsToContents()
        self.table_widget.setWordWrap(True)

    
    def reload_file(self):
        '''Handles selection of a new file'''
        filenames = QFileDialog.getOpenFileNames(
            self, self.tr('Load from Files'), '.', self.tr('log files {.log} (*.log)'))
        if filenames[0] == []:
            return 0
        file = open(filenames[0][0], 'r')
        lines = file.read().splitlines()
        self.file = lines
        self.condensed_layout()
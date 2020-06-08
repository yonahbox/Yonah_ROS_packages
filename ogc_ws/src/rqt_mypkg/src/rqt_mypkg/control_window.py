#!/usr/bin/env python3
import os
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QLabel, QProgressBar, QPlainTextEdit
from python_qt_binding.QtWidgets import QScrollArea, QPushButton, QVBoxLayout, QCheckBox, QHBoxLayout
from python_qt_binding.QtWidgets import QAction, QTreeWidget, QTreeWidgetItem, QMessageBox
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
import __main__
from checklist_window import ChecklistWindow

class ControlWindow(QWidget):
    def __init__(self):
        super(ControlWindow, self).__init__()
        self.setWindowTitle("Ground Control Station")
        self.resize(500, 700)
        self.move(200,100)
        
        # create the layout
        self.layout = QHBoxLayout(self)
        self.buttons_layout = QVBoxLayout(self)
        self.progressbar_layout = QVBoxLayout(self)
        
        # create the widgets
        self.create_widget()
        self.has_message_opened = 0

        # add the widgets into the layouts
        self.layout.addLayout(self.progressbar_layout)
        self.layout.addLayout(self.buttons_layout)
        self.setLayout(self.layout)

    def create_widget(self):
        # declare buttons and connect each of them to a function
        self.load_button = QLabel('Load')
        self.ok_button = QLabel('OK')
        self.cancel_button = QLabel('Cancel')
        self.load_button2 = QPushButton('Load')
        self.ok_button2 = QPushButton('OK')
        self.cancel_button2 = QPushButton('Cancel')
        
        for i in range(3):
            self.create_progress(i)

        self.buttons_layout.addWidget(self.load_button)
        self.buttons_layout.addWidget(self.cancel_button)
        self.buttons_layout.addWidget(self.ok_button)

    def create_progress(self, aircraft_no):
        self.waypoint_layout = QVBoxLayout(self)
        self.waypoint_header_layout = QHBoxLayout(self) #waypoint_header will be nested inside waypoint_layout with the progress bar beneath it
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.aircraft_waypoint_textedit = QPlainTextEdit()
        self.aircraft_waypoint_textedit.setMaximumHeight(40)
        self.aircraft_progressbar = QProgressBar()

        self.waypoint_header_layout.addWidget(self.aircraft_label)
        self.waypoint_header_layout.addWidget(self.aircraft_waypoint_textedit)
        self.waypoint_layout.addLayout(self.waypoint_header_layout)
        self.waypoint_layout.addWidget(self.aircraft_progressbar)
        self.progressbar_layout.addLayout(self.waypoint_layout)
        
    
    def shutdown(self):
        self.close()
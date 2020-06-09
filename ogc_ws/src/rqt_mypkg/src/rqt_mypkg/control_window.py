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
        self.adjustSize()
        self.move(200,100)
        
        self.waypoint_plaintext_dict = {}
        # create the layout
        self.layout = QHBoxLayout(self)
        self.buttons_layout = QVBoxLayout(self)
        self.progressbar_layout = QVBoxLayout(self)
        
        # create the widgets
        self.active_aircrafts = 20
        for i in range(1,self.active_aircrafts):
            self.create_progressbar(i)

        # add the widgets into the layouts
        self.layout.addLayout(self.progressbar_layout)
        self.setLayout(self.layout)

    # def create_widget(self):
    #     self.waypoint_plaintext_dict.get('aircraft' + str(1)).appendPlainText('test')
    #     self.waypoint_plaintext_dict.get('aircraft' + str(2)).appendPlainText('testing')
    #     print(self.waypoint_plaintext_dict)

    def create_progressbar(self, aircraft_no):
        self.waypoint_layout = QVBoxLayout(self)
        self.waypoint_header_layout = QHBoxLayout(self) # waypoint_header_layout will be nested inside waypoint_layout with the progress bar beneath it
        
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.waypoint_plaintext_dict['aircraft' + str(aircraft_no)] = QPlainTextEdit()
        self.waypoint_plaintext_dict.get('aircraft' + str(aircraft_no)).setMaximumHeight(40)
        # self.aircraft_waypoint_textedit = QPlainTextEdit()
        # self.aircraft_waypoint_textedit.setMaximumHeight(40)
        self.waypoint_plaintext_dict['progress_bar_aircraft' + str(aircraft_no)] = QProgressBar()

        self.waypoint_header_layout.addWidget(self.aircraft_label)
        self.waypoint_header_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + str(aircraft_no)])
        self.waypoint_layout.addLayout(self.waypoint_header_layout)
        self.waypoint_layout.addWidget(self.waypoint_plaintext_dict['progress_bar_aircraft' + str(aircraft_no)])
        self.progressbar_layout.addLayout(self.waypoint_layout)
    
    def shutdown(self):
        self.close()
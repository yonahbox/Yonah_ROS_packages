#!/usr/bin/env python3
import os
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QLabel, QProgressBar, QPlainTextEdit
from python_qt_binding.QtWidgets import QScrollArea, QPushButton, QVBoxLayout, QCheckBox, QHBoxLayout
from python_qt_binding.QtWidgets import QAction, QTreeWidget, QTreeWidgetItem, QMessageBox
from PyQt5.QtWidgets import *

from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
import __main__
from checklist_window import ChecklistWindow

class SummaryWindow(QWidget):
    def __init__(self):
        super(SummaryWindow, self).__init__()
        self.setWindowTitle("Summary Page")
        self.move(200,100)
        
        self.waypoint_plaintext_dict = {}
        # create the layout
        self.layout = QVBoxLayout(self)
        self.summary_layout = QVBoxLayout(self)
        
        # create the widgets
        summarised_fields = ['MODE', 'STATUS', 'AIRSPEED', 'ALTITUDE']
        self.active_aircrafts = 5
        for i in range(1, self.active_aircrafts):
            self.create_summary(i, summarised_fields)

        self.statustext = QPlainTextEdit()
        # add the widgets into the layouts
        self.layout.addWidget(self.statustext)
        self.layout.addLayout(self.summary_layout)
        self.setLayout(self.layout)

    # def create_widget(self):
    #     self.waypoint_plaintext_dict.get('aircraft' + str(1)).appendPlainText('test')
    #     self.waypoint_plaintext_dict.get('aircraft' + str(2)).appendPlainText('testing')
    #     print(self.waypoint_plaintext_dict)

    def create_summary(self, aircraft_no, summarised_fields):
        self.summary_details_layout = QVBoxLayout(self)
         # summary_fields_layout will be nested inside summary_details_layout with the progress bar beneath it
        
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.aircraft_label.setContentsMargins(0, 30, 0, 0)
        print(self.aircraft_label.setAlignment(Qt.AlignLeft))
        self.summary_details_layout.addWidget(self.aircraft_label)
        # @TODO I think the generation of the sub-fields can be automated and the setmaxheight can be condensed
        for i in summarised_fields:
            # if summarised_fields.index(i) 
            self.summary_fields_layout = QHBoxLayout(self)
            self.subfield_label_mode = QLabel(i)
            self.subfield_label_mode.setFixedSize(80,20)
            self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)

            self.summary_fields_layout.addWidget(self.subfield_label_mode)
            self.summary_fields_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)])
            self.summary_details_layout.addLayout(self.summary_fields_layout)

        self.summary_layout.addLayout(self.summary_details_layout)
    
    def shutdown(self):
        self.close()
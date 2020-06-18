#!/usr/bin/env python3
import os
import rospkg
import __main__

from python_qt_binding import loadUi
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from checklist_window import ChecklistWindow

# File is still changing rapidly and dynamically, hence comments might not be accurate
# @TODO change the variable names. As of now it is heavily referenced from the summary page
# AircraftInfo is the parent widget for all the other aircrafts
class AircraftInfo(QWidget):
    def __init__(self, no_aircraft):
        super(AircraftInfo, self).__init__()
        self.setWindowTitle("Summary Page")
        
        self.waypoint_plaintext_dict = {}
        # create the layout
        self.main_layout = QVBoxLayout()
        self.summary_layout = QVBoxLayout()
        self.setLayout(self.main_layout)
        
        # create the widgets
        summarised_fields = [
            'Mode', 
            'Status', 
            'Altitude', 
            'Airspeed',  
            'Groundspeed', 
            'GPS Coordinate', 
            'Throttle', 
            'VTOL Status', 
            'Windspeed',
            'Target Waypoint',
            'Flying Time',
            'Vibe status',
            'Fuel Level',
            'Quad Battery']

        self.active_aircrafts = no_aircraft
        self.create_summary(self.active_aircrafts, summarised_fields)
        self.statustext_label = QLabel('Status Text')
        self.statustext_label.setContentsMargins(0, 30, 0, 0)
        self.statustext = QPlainTextEdit()
        self.statustext.setReadOnly(True)
        self.statustext.setMinimumHeight(300)

        # Add the widgets into the layouts
        self.main_layout.addLayout(self.summary_layout)
        self.main_layout.addWidget(self.statustext_label)
        self.main_layout.addWidget(self.statustext)

    def create_summary(self, aircraft_no, summarised_fields):
        self.summary_details_layout = QVBoxLayout()
         # summary_fields_layout will be nested inside summary_details_layout with the progress bar beneath it
        self.aircraft_label = QLabel('Aircraft ' + str(aircraft_no))
        self.summary_details_layout.addWidget(self.aircraft_label)
        for i in summarised_fields:
            self.summary_fields_layout = QHBoxLayout()
            self.subfield_label_mode = QLabel(i)
            self.subfield_label_mode.setFixedSize(120,30)
            # Create an entry in the dictionary with name aircraftMODE1 and set attributes of the name
            self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)] = QPlainTextEdit()
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setMaximumHeight(40)
            self.waypoint_plaintext_dict.get('aircraft' + i + str(aircraft_no)).setReadOnly(True)

            self.summary_fields_layout.addWidget(self.subfield_label_mode)
            self.summary_fields_layout.addWidget(self.waypoint_plaintext_dict['aircraft' + i + str(aircraft_no)])
            self.summary_details_layout.addLayout(self.summary_fields_layout)

        self.summary_layout.addLayout(self.summary_details_layout)

    def shutdown(self):
        self.close()

# The child classes as of now are empty -- meaning they do not modify anything from the parent class
# This serves as a placeholder in the case we need to modify anything specific to an aircraft, then we can do it easily
class Aircraft1(AircraftInfo):
    pass
class Aircraft2(AircraftInfo):
    pass
class Aircraft3(AircraftInfo):
    pass
class Aircraft4(AircraftInfo):
    pass
class Aircraft5(AircraftInfo):
    pass
class Aircraft6(AircraftInfo):
    pass
class Aircraft7(AircraftInfo):
    pass
class Aircraft8(AircraftInfo):
    pass
class Aircraft9(AircraftInfo):
    pass
class Aircraft10(AircraftInfo):
    pass
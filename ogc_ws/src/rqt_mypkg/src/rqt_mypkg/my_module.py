#!/usr/bin/env python3
'''
Copyright (C) 2020 Dani Purwadi

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
import rospy
import rospkg
import rosservice
import rostopic

from std_msgs.msg import String 
from mavros_msgs.msg import StatusText, State, VFR_HUD, WaypointReached, WaypointList
from sensor_msgs.msg import NavSatFix
# from despatcher.msg import RegularPayload
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot, QAbstractListModel, QObject, pyqtSignal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from PyQt5.QtWidgets import *
from python_qt_binding.QtSvg import QSvgGenerator
from checklist_window import ChecklistWindow
from control_window import WaypointWindow
from summary_window import SummaryWindow
from command_window import CommandWindow
from aircraft_info import *

#[DA] Class MyPlugin inherits Plugin and Plugin is qt_gui.plugin.Plugin
class MyPlugin(Plugin):

    def __init__(self, context): #[DA] the class MyPlugin takes in context as a variable
        #[DA] super means it inherits the parent class' init() property
        #[DA] In this case the parent class requires context as a parameter for its init
        super(MyPlugin, self).__init__(context) ### this syntax means that it is a python 2 syntax and not python 3
        # from argparse import ArgumentParser
        # args = self._parse_args(context.argv()) #[DA] Need to investigate what parse args does

        # Give QObjects reasonable names
        self.setObjectName('main_window') #this is inherited from QObject class which sets name of the object
        self._widget = QWidget() # this defines the attribute of MyPlugin class. QWidget is a method in QtWidget
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'second_window.ui')
        
        # Extend the widget with all attributes and children from UI file
        # there is actually a third param available if we have custom class in our widget, but I think we can ignore this
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('MainWindowUI') #[DA] This sets the name instance using property inherited by _widget
        self._widget.setWindowTitle('Yonah RQt')

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number():
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        # Get the number of active aircrafts here
        self.active_aircrafts = 5
        self.aircrafts_info = {}
        for i in range (self.active_aircrafts + 1):
            self.aircrafts_info['AC' + str(i)] = Aircraft1(i)

        # Declare variables for each imported class
        self.ChecklistWindow = ChecklistWindow()
        self.WaypointWindow = WaypointWindow()
        self.SummaryWindow = SummaryWindow()
        self.CommandWindow = CommandWindow()

        # Create layout for Waypoint scroll window
        self.scroll = QScrollArea()
        self.scroll.setMinimumHeight(700)
        self.scroll.setMinimumWidth(600)
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.WaypointWindow)

        self.create_tab_windows()

        # Add both layouts into the main layout
        
        self._widget.verticalLayout2.addWidget(self.tab)
        self._widget.verticalLayout2.addWidget(self.CommandWindow)

        # Keep track whether the checklist window is opened
        self.checklist_opened = 0

        # Connect each button to a function
        # self._widget.arming_pushbutton.pressed.connect(self.arming)
        # self._widget.control_pushbutton.pressed.connect(self.transfer_control)
        # self._widget.mode_manual_pushbutton.pressed.connect(self.mode_manual)
        # self._widget.mode_auto_pushbutton.pressed.connect(self.mode_auto)
        # self._widget.load_mission_pushbutton.pressed.connect(self.load_mission)
        # self._widget.check_mission_pushbutton.pressed.connect(self.check_mission)
        # self._widget.checklist_pushbutton.pressed.connect(self.ChecklistWindow.show)
        
        #Subscriber lists
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.status_text)
        # rospy.Subscriber("ogc/from_despatcher/regular", RegularPayload, self.regular_payload)
        rospy.Subscriber('ogc/from_despatcher/ondemand', String, self.ondemand)
        rospy.Subscriber("mavros/state", State, self.mode_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.VFR_HUD)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoint_total)
        
        self.trigger = Signal(str)
        # Publisher List
        self.arming_publisher = rospy.Publisher('ogc/to_despatcher', String, queue_size = 5)
        self.transfer_control_publisher = rospy.Publisher('transfer_control', String, queue_size = 5)
        self.mode_publisher = rospy.Publisher('mode_change', String, queue_size = 5)
        self.mission_publisher = rospy.Publisher('load_mission', String, queue_size = 5)

        self.rate = rospy.Rate(2)
        context.add_widget(self._widget)

    def create_tab_windows(self):
        # Create layout for Summary scroll window
        self.summary_scroll = QScrollArea()
        self.summary_scroll.setMinimumHeight(500)
        self.summary_scroll.setMinimumWidth(600)
        self.summary_scroll.setWidgetResizable(True)
        self.tab = QTabWidget()
        self.summary_scroll.setWidget(self.SummaryWindow)
        self.tab.addTab(self.summary_scroll, 'Summary')
        for i in range (1, self.active_aircrafts + 1):
            key = 'aircraft' + str(i) + ' scroll'
            self.aircrafts_info[key] = QScrollArea()
            self.aircrafts_info.get(key).setMinimumHeight(500)
            self.aircrafts_info.get(key).setMinimumWidth(600)
            self.aircrafts_info.get(key).setWidgetResizable(True)
            self.aircrafts_info.get(key).setWidget(self.aircrafts_info.get('AC' + str(i)))
            self.tab.addTab(self.aircrafts_info.get(key), 'Aircraft ' + str(i))
            
        self.tab.setMinimumHeight(500)
        self._widget.verticalLayout.addStretch(1)
        self._widget.verticalLayout.addWidget(self.scroll)

    # Create a signal-slot mechanism for each function in order to display information
    # @TODO optimize the code for the signal slot such that only one function is needed
    def ondemand(self, data):
        status = Communicate()
        status.ondemand_signal.connect(self.ondemand_display)
        status.ondemand_signal.emit(data.text)

    def regular_payload(self, data):
        status = Communicate()
        status.string_signal.connect(self.airspeed_display)
        status.string_signal.emit(data.airspeed)
        status.string_signal.connect(self.altitude_display)
        status.string_signal.emit(data.altitude)
        status.string_signal.connect(self.mode_status_display) 
        status.string_signal.emit(data.armed)
        status.string_signal.connect(self.groundspeed_display)
        status.string_signal.emit(data.groundspeed)
        status.string_signal.connect(self.throttle_display)
        status.string_signal.emit(data.throttle)
        status.string_signal.connect(self.gps_display)
        status.string_signal.emit(data.lat, data.lon)
        status.string_signal.connect(self.windspeed_display)
        status.string_signal.emit(data.windspeed)
        status.string_signal.connect(self.vibe_display)
        status.string_signal.emit(data.vibe)
        status.string_signal.connect(self.vtol_display)
        status.string_signal.emit(data.vtol)
        status.string_signal.connect(self.waypoint_display)
        status.string_signal.emit(data.text)
        status.string_signal.connect(self.time_display)
        status.string_signal.emit(data.text)
        status.string_signal.connect(self.fuel_display)
        status.string_signal.emit(data.fuel)
        status.string_signal.connect(self.quad_batt_display)
        status.string_signal.emit(data.quad_batt)
        
    def status_text(self, data):
        status = Communicate()
        status.string_signal.connect(self.status_text_display)
        status.string_signal.emit(data.text)
    
    def mode_status(self, data):
        status = Communicate()
        status.string_signal.connect(self.mode_status_display)
        status.string_signal.emit(data.mode)

        status.string_signal.connect(self.arm_status_display)
        status.string_signal.emit(str(data.armed))

    def VFR_HUD(self, data):
        status = Communicate()
        status.airspeed_signal.connect(self.airspeed_display)
        status.airspeed_signal.emit(data.airspeed)
        status.altitude_signal.connect(self.altitude_display)
        status.altitude_signal.emit(data.altitude)

    def waypoint_total(self, data):
        status = Communicate()
        status.waypoint_list_signal.connect(self.waypoint_total_display)
        status.waypoint_list_signal.emit(data.waypoints, data.current_seq)
    
    # All functions with _display are the functions that takes the information and display it to the UI
    def groundspeed_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftGroundspeed1').setPlainText(data)
    
    def throttle_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftThrottle1').setPlainText(data)

    def gps_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftGPS1').setPlainText(data)

    def windspeed_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftWindspeed1').setPlainText(data)

    def vibe_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftVibe Status1').setPlainText(data)

    def vtol_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftVTOL Status1').setPlainText(data)

    def waypoint_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftTarget Waypoint1').setPlainText(data)

    def time_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftFlying Time1').setPlainText(data)

    def fuel_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftFuel Level1').setPlainText(data)

    def quad_batt_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftQuad Battery1').setPlainText(data)
    
    def mode_status_display(self, mode_status):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftMODE1').setPlainText(mode_status)

    def altitude_display(self, altitude):
        altitude = str(round(altitude, 1)) + ' m'
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftALTITUDE1').setPlainText(altitude)        
        # self._widget.altitude_textedit.setText(altitude)

    def airspeed_display(self, airspeed):
        airspeed = str(round(airspeed, 1)) + ' m/s'
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftAIRSPEED1').setPlainText(airspeed)        
        # self._widget.airspeed_textedit.setText(airspeed)

    def waypoint_total_display(self, total, sequence):
        total = len(total) - 1
        # self._widget.progressBar.setRange(0,total)
        # self._widget.progressBar.setValue(sequence)
        # self._widget.wp_textedit.setText('Current WP: ' + str(sequence) + ' out of ' + str(total))

    
    def arm_status_display(self, arm_status):
        if arm_status == 'False':
            text_to_display = 'DISARMED'
        else:
            text_to_display = 'ARMED'
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftSTATUS1').setPlainText(text_to_display)        
        # self._widget.arming_textedit.setText(str(text_to_display))

    # Send commands to air_despatcher
    def arming (self):
        print(self.checklist_opened)
        self.arming_publisher.publish('arm')
        # self.status_text_display('Arming message sent')
        # if self.text_to_display == 'DISARMED':
        #     self.arming.publish('ARM')
        # elif self.text_to_display == 'ARMED':
        #     self.arming.publish('DISARM')
        #     print('Now disarm it')

    # Close all windows
    # @TODO close all ROS connections as well (unsubscribe from the channels)
    def shutdown_plugin(self):
        self.ChecklistWindow.shutdown()
        self.WaypointWindow.shutdown()
        self.SummaryWindow.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
   
# Class that is responsible for signal and slot function
class Communicate (QObject):
    # technically, all the signals that has the same input, such as string
    # can be combined into one variable. However, for clarity purpose,
    # I split each variable for each display we need to make
    # regular_paylaod_signal = Signal(RegularPayload)
    string_signal = Signal(str)
    boolean_signal = Signal(bool)
    float_signal = Signal(float)
    
    status_text_signal = Signal(str)
    arm_signal = Signal(bool)
    mode_signal = Signal(str)
    altitude_signal = Signal(float)
    airspeed_signal = Signal(float)
    waypoint_list_signal = Signal(list, int)
    waypoint_index_signal = Signal(int)



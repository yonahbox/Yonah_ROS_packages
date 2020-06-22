#!/usr/bin/env python3
'''
my_module: Main Python file where RQt layout and components are assembled

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
import rospy
import rospkg
import rosservice
import rostopic

from std_msgs.msg import String 
from mavros_msgs.msg import StatusText, State, VFR_HUD, WaypointReached, WaypointList
from sensor_msgs.msg import NavSatFix
from despatcher.msg import RegularPayload
from despatcher.msg import LinkMessage
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot, QAbstractListModel, QObject, pyqtSignal
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from PyQt5.QtWidgets import *
from python_qt_binding.QtSvg import QSvgGenerator
from checklist_window import ChecklistWindow
from waypoint_window import WaypointWindow
from summary_window import SummaryWindow
from command_window import CommandWindow
from aircraft_info import *

#[DA] Class MyPlugin inherits Plugin and Plugin is qt_gui.plugin.Plugin
class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('main_window')
        self._widget = QWidget()
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'second_window.ui')
        
        # Extend the widget with all attributes and children from UI file
        # there is actually a third param available if we have custom class in our widget
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('MainWindowUI')
        self._widget.setWindowTitle('Yonah RQt')

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to 
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
        self.WaypointWindow = WaypointWindow(self.active_aircrafts)
        self.SummaryWindow = SummaryWindow(self.active_aircrafts)
        self.CommandWindow = CommandWindow(self.active_aircrafts)

        # Create layout for Waypoint scroll window
        self.scroll = QScrollArea()
        self.scroll.setMinimumHeight(700)
        self.scroll.setMinimumWidth(600)
        self.scroll.setWidgetResizable(True)
        self.scroll.setWidget(self.WaypointWindow)
        self._widget.verticalLayout.addWidget(self.scroll)
        # Create the tab windows for the aircraft-specific information
        self.create_tab_windows()

        # Add both layouts into the main layout
        self._widget.verticalLayout2.addWidget(self.tab)
        self._widget.verticalLayout2.addWidget(self.CommandWindow)

        # Keep track whether the checklist window is opened
        self.checklist_opened = 0

        # Connect each button to a function
        self.CommandWindow.arm_button.pressed.connect(self.arming)
        self.CommandWindow.go_button.pressed.connect(self.go_button)
        # self.CommandWindow.mission_check_button.pressed.connect(self.check_mission)
        # self.CommandWindow.mission_load_button.pressed.connect(self.load_mission)
        self.CommandWindow.checklist_button.pressed.connect(self.ChecklistWindow.show)
        
        #Subscriber lists
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.status_text)
        rospy.Subscriber("ogc/from_despatcher/regular", RegularPayload, self.regular_payload)
        rospy.Subscriber("ogc/yonahtext", String, self.status_text)
        rospy.Subscriber("ogc/from_despatcher/ondemand", String, self.ondemand)
        rospy.Subscriber("mavros/state", State, self.mode_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.regular_payload)
        # rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoint_total)

        # Publisher List
        self.command_publisher = rospy.Publisher('ogc/to_despatcher', LinkMessage, queue_size = 5)
        # self.mission_publisher = rospy.Publisher('load_mission', String, queue_size = 5)

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
        print('on demand')
        status = Communicate()
        status.ondemand_signal.connect(self.ondemand_display)
        status.ondemand_signal.emit(str(data))

    def regular_payload(self, data):
        status = Communicate()
        status.airspeed_signal.connect(self.airspeed_display)
        status.airspeed_signal.emit(data.airspeed, '1')
        status.alt_signal.connect(self.altitude_display)
        status.alt_signal.emit(data.alt, '1')
        status.mode_signal.connect(self.mode_status_display) 
        status.mode_signal.emit(data.mode, '1')
        status.arm_signal.connect(self.arm_status_display) 
        status.arm_signal.emit(data.armed, '1')
        status.groundspeed_signal.connect(self.groundspeed_display)
        status.groundspeed_signal.emit(data.groundspeed)
        status.throttle_signal.connect(self.throttle_display)
        status.throttle_signal.emit(data.throttle)
        status.gps_signal.connect(self.gps_display)
        status.gps_signal.emit(data.lat, data.lon)
        status.vibe_signal.connect(self.vibe_display)
        status.vibe_signal.emit(data.vibe)
        status.vtol_signal.connect(self.vtol_display)
        status.vtol_signal.emit(data.vtol)
        status.wp_signal.connect(self.waypoint_display)
        status.wp_signal.emit(data.wp)
        status.time_signal.connect(self.time_display)
        status.time_signal.emit(data.header.stamp.secs)
        status.fuel_signal.connect(self.fuel_display)
        status.fuel_signal.emit(data.fuel)
        status.batt_signal.connect(self.quad_batt_display)
        status.batt_signal.emit(data.batt)
        
    def status_text(self, data):
        print('status')
        status = Communicate()
        status.ondemand_signal.connect(self.status_text_display)
        status.ondemand_signal.emit(data.text)
    
    def mode_status(self, data):
        status = Communicate()
        status.mode_signal.connect(self.mode_status_display)
        status.mode_signal.emit(data.mode, '1')
        status.arm_signal.connect(self.arm_status_display)
        status.arm_signal.emit(data.armed, '1')

    def VFR_HUD(self, data):
        status = Communicate()
        status.float_signal.connect(self.airspeed_display)
        status.float_signal.emit(data.airspeed)
        status.float_signal.connect(self.altitude_display)
        status.float_signal.emit(data.altitude)

    def waypoint_total(self, data):
        status = Communicate()
        status.waypoint_list_signal.connect(self.waypoint_total_display)
        status.waypoint_list_signal.emit(data.waypoints, data.current_seq)
    
    # All functions with _display are the functions that takes the information and display it to the UI
    def groundspeed_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftGroundspeed1').setPlainText(data)
    
    def throttle_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftThrottle1').setPlainText(data)

    def gps_display(self, lat, lon):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftGPS1').setPlainText(str(lat + lon))

    def vibe_display(self, data):
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftVibe Status1').setPlainText(str(data))

    def vtol_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftVTOL Status1').setPlainText(data)

    def waypoint_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftTarget Waypoint1').setPlainText(data)

    def time_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftFlying Time1').setPlainText(data)

    def fuel_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftFuel Level1').setPlainText(data)

    def quad_batt_display(self, data):
        data = str(data)
        self.aircrafts_info.get('AC1').waypoint_plaintext_dict.get('aircraftQuad Battery1').setPlainText(data)
    
    def mode_status_display(self, mode_status, id):
        mode_status = str(mode_status)
        self.aircrafts_info.get('AC' + id).waypoint_plaintext_dict.get('aircraftMode' + id).setPlainText(mode_status)
        self.SummaryWindow.waypoint_plaintext_dict.get('aircraftMode' + id).setPlainText(mode_status)

    def altitude_display(self, altitude, id):
        id = str(id)
        altitude = str(round(altitude, 1)) + ' m'
        self.aircrafts_info.get('AC' + id).waypoint_plaintext_dict.get('aircraftAltitude' + id).setPlainText(altitude)
        self.SummaryWindow.waypoint_plaintext_dict.get('aircraftAltitude' + id).setPlainText(altitude)        

    def airspeed_display(self, airspeed, id):
        id = str(id)
        airspeed = str(round(airspeed, 1)) + ' m/s'
        self.aircrafts_info.get('AC' + id).waypoint_plaintext_dict.get('aircraftAirspeed' + id).setStyleSheet("Color: rgb(255, 0, 0);")     
        self.aircrafts_info.get('AC' + id).waypoint_plaintext_dict.get('aircraftAirspeed' + id).setPlainText(airspeed)
        self.SummaryWindow.waypoint_plaintext_dict.get('aircraftAirspeed' + id).setPlainText(airspeed)                

    def waypoint_total_display(self, total, sequence):
        total = len(total) - 1
        # self._widget.progressBar.setRange(0,total)
        # self._widget.progressBar.setValue(sequence)
        # self._widget.wp_textedit.setText('Current WP: ' + str(sequence) + ' out of ' + str(total))
    
    def arm_status_display(self, arm_status, id):
        if arm_status == 'False':
            text_to_display = 'DISARMED'
        else:
            text_to_display = 'ARMED'
        self.aircrafts_info.get('AC' + id).waypoint_plaintext_dict.get('aircraftStatus' + id).setPlainText(text_to_display)        
        self.SummaryWindow.waypoint_plaintext_dict.get('aircraftStatus' + id).setPlainText(text_to_display)        
    
    def status_text_display(self, status_text):
        self.SummaryWindow.statustext.appendPlainText(status_text)
        self.aircrafts_info.get('AC1').statustext.appendPlainText(status_text)

    def ondemand_display(self, data):
        self.SummaryWindow.statustext.appendPlainText(str(data))
        self.aircrafts_info.get('AC1').statustext.appendPlainText(data)

    # Send commands to air_despatcher
    def arming (self):
        message = LinkMessage()
        message.id = 1
        message.data = 'arm'
        self.command_publisher.publish(message)
        # self.status_text_display('Arming message sent')
        # if self.text_to_display == 'DISARMED':
        #     self.arming.publish('ARM')
        # elif self.text_to_display == 'ARMED':
        #     self.arming.publish('DISARM')
        #     print('Now disarm it')

    def go_button(self):
        message = LinkMessage()
        message.id = 1
        message.data = 'mode 5'
        self.command_publisher.publish(message)
        print('go')

    # Close all windows
    # TODO close all ROS connections as well (unsubscribe from the channels)
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
    # regular_signal = Signal(RegularPayload)
    # boolean_signal = Signal(bool)
    # float_signal = Signal(float, int)
    # int_signal = Signal(int)
    # string_signal = Signal(str)
    airspeed_signal = Signal(float, str)
    alt_signal = Signal(float, str)
    arm_signal = Signal(bool, str)
    status_text_signal = Signal(str)
    batt_signal = Signal(int)
    fuel_signal = Signal(int)
    groundspeed_signal = Signal(int)
    gps_signal = Signal(float, float)    
    mode_signal = Signal(int, str)
    throttle_signal = Signal(int)
    vibe_signal = Signal(int)
    vtol_signal = Signal(int)
    wp_signal = Signal(int)
    time_signal = Signal(int)

    ondemand_signal = Signal(str)
    waypoint_list_signal = Signal(list, int)
    waypoint_index_signal = Signal(int)


def color(color):
    if color == 'red':
        return QColor(255,0,0)
    elif color == 'black':
        return QColor(0,0,0)
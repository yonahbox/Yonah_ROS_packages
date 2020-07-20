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
import regular

from std_msgs.msg import String 
from mavros_msgs.msg import StatusText, State, VFR_HUD, WaypointList
from despatcher.msg import RegularPayload, LinkMessage
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtCore import Qt, Signal, Slot, QObject
from PyQt5.QtWidgets import QWidget, QTabWidget, QScrollArea, QShortcut
from .checklist_window import ChecklistWindow
from .waypoint_window import WaypointWindow
from .summary_window import SummaryWindow
from .command_window import CommandWindow
from .popup_window import PopupMessages
from .aircraft_info import AircraftInfo

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName("main_window")
        self._widget = QWidget()
        self._widget.setMinimumSize(1280, 800)
        self._widget.setWindowTitle("Yonah RQt")
        context.add_widget(self._widget)

        # Get path to UI file and load it
        ui_file = os.path.join(rospkg.RosPack().get_path("rqt_mypkg"), "resource", "second_window.ui")
        loadUi(ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number():
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        
        # Declare attributes
        self.time = 0
        self.destination_id = 1
        self.aircraft_list = []
        self.aircrafts_info = {}
        self.checklist_info = {}
        
        # Declare attributes for each imported class
        self.PopupMessages = PopupMessages()
        self.WaypointWindow = WaypointWindow(self.aircraft_list)
        self.SummaryWindow = SummaryWindow(self.aircraft_list)
        self.CommandWindow = CommandWindow(self.aircraft_list)

        self.create_layout()
        self.shortcuts()

        # Subscriber lists
        rospy.Subscriber("ogc/from_despatcher/regular", RegularPayload, self.regular_payload)
        rospy.Subscriber("ogc/yonahtext", String, self.status_text)
        rospy.Subscriber("ogc/from_despatcher/ondemand", String, self.ondemand)

        rospy.Subscriber("mavros/statustext/recv", StatusText, self.status_text)
        rospy.Subscriber("mavros/state", State, self.mode_status_sitl)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.VFR_HUD_sitl)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoint_sitl)

        # Publisher List
        self.command_publisher = rospy.Publisher("ogc/to_despatcher", LinkMessage, queue_size = 5)

    def create_layout(self):
        '''Populate the UI with the modules from other files'''
        # Create layout for Waypoint scroll window
        scroll = QScrollArea()
        scroll.setMinimumHeight(800)
        scroll.setMinimumWidth(600)
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.WaypointWindow)
    
        # Create the tab windows for the aircraft-specific information
        self.create_tab_windows()
        self.tab.currentChanged.connect(self.tab_change)

        # Add all 3 layouts into the main layout
        self._widget.verticalLayout.addWidget(scroll)
        self._widget.verticalLayout2.addWidget(self.tab)
        self._widget.verticalLayout2.addWidget(self.CommandWindow)

    def create_tab_windows(self):
        '''Create layout for Summary scroll window'''
        summary_scroll = QScrollArea()
        summary_scroll.setMinimumHeight(500)
        summary_scroll.setMinimumWidth(600)
        summary_scroll.setWidgetResizable(True)
        summary_scroll.setWidget(self.SummaryWindow)

        self.tab = QTabWidget()
        self.tab.addTab(summary_scroll, "Summary")

        if self.aircraft_list == []:
            active_aircrafts = [x for x in range (1,10)]
            self.SummaryWindow.create_layout(active_aircrafts)
            self.CommandWindow.create_combobox(active_aircrafts)
            self.WaypointWindow.create_layout(active_aircrafts)
        else:
            active_aircrafts = self.aircraft_list

        for i in active_aircrafts:
            self.aircrafts_info["AC" + str(i)] = AircraftInfo(i)
            self.checklist_info["AC" + str(i)] = ChecklistWindow(i)

            self.key = "aircraft" + str(i) + " scroll"
            self.aircrafts_info[self.key] = QScrollArea()
            self.aircrafts_info.get(self.key).setMinimumHeight(500)
            self.aircrafts_info.get(self.key).setMinimumWidth(600)
            self.aircrafts_info.get(self.key).setWidgetResizable(True)
            self.aircrafts_info.get(self.key).setWidget(self.aircrafts_info.get("AC" + str(i)))
            self.tab.addTab(self.aircrafts_info.get(self.key), "Aircraft " + str(i))
        self.tab.setMinimumHeight(500)
        
    def update_active_aircrafts(self, active_aircrafts):
        self.SummaryWindow.remove(self.SummaryWindow.summary_layout)
        self.SummaryWindow.create_layout(active_aircrafts)
        self.CommandWindow.create_combobox(active_aircrafts)
        self.WaypointWindow.remove(self.WaypointWindow.progressbar_layout)
        self.WaypointWindow.create_layout(active_aircrafts)

        # remove all the tabs
        for i in range(self.tab.count(), 0, -1):
            self.tab.removeTab(i)
        # add back all the tabs
        for j in active_aircrafts:
            if self.aircrafts_info.get(self.key) == None:
                self.aircrafts_info["AC" + str(j)] = AircraftInfo(j)
                self.checklist_info["AC" + str(j)] = ChecklistWindow(j)

                self.aircrafts_info[self.key] = QScrollArea()
                self.aircrafts_info.get(self.key).setMinimumHeight(500)
                self.aircrafts_info.get(self.key).setMinimumWidth(600)
                self.aircrafts_info.get(self.key).setWidgetResizable(True)
                self.aircrafts_info.get(self.key).setWidget(self.aircrafts_info.get("AC" + str(j)))
            
            self.tab.addTab(self.aircrafts_info.get(self.key), "Aircraft " + str(j))

    def tab_change(self, i):
        '''Changes the command_window drop-down menu to follow the change in tab'''
        if i == 0: # When Tab is at Summary Page, show AC 1 in the Command Window combo_box
            i = 1
        self.CommandWindow.combo_box.setCurrentIndex(i - 1)

    ################################
    # Create Signal Slot functions #
    ################################
    def regular_payload(self, data):
        aircraft_id = str(data.vehicle_no)
        if aircraft_id not in self.aircraft_list:
            self.aircraft_list.append(aircraft_id)
            self.aircraft_list.sort()
            self.update_active_aircrafts(self.aircraft_list)
        status = Communicate()
        status.airspeed_signal.connect(self.airspeed_display)
        status.airspeed_signal.emit(data.airspeed, aircraft_id)
        status.alt_signal.connect(self.altitude_display)
        status.alt_signal.emit(data.alt, aircraft_id)
        status.arm_signal.connect(self.arm_status_display) 
        status.arm_signal.emit(data.armed, aircraft_id)
        status.batt_signal.connect(self.quad_batt_display)
        status.batt_signal.emit(data.batt, aircraft_id)
        status.fuel_signal.connect(self.fuel_display)
        status.fuel_signal.emit(data.fuel, aircraft_id)
        status.groundspeed_signal.connect(self.groundspeed_display)
        status.groundspeed_signal.emit(data.groundspeed, aircraft_id)
        status.gps_signal.connect(self.gps_display)
        status.gps_signal.emit(data.lat, data.lon, aircraft_id)
        status.mode_signal.connect(self.mode_status_display) 
        status.mode_signal.emit(data.mode, aircraft_id)
        status.throttle_signal.connect(self.throttle_display)
        status.throttle_signal.emit(data.throttle, aircraft_id)
        status.vibe_signal.connect(self.vibe_display)
        status.vibe_signal.emit(data.vibe, aircraft_id)
        status.vtol_signal.connect(self.vtol_display)
        status.vtol_signal.emit(data.vtol, aircraft_id)
        status.wp_signal.connect(self.waypoint_display)
        status.wp_signal.emit(data.wp, aircraft_id)
        status.time_signal.connect(self.time_display)
        status.time_signal.emit(data.header.stamp.secs, aircraft_id)

    def ondemand(self, data):
        status = Communicate()
        status.ondemand_signal.connect(self.ondemand_display)
        status.ondemand_signal.emit(str(data), "1")

    def status_text(self, data):
        status = Communicate()
        status.ondemand_signal.connect(self.status_text_display)
        status.ondemand_signal.emit(data.text, "1")
    
    ####### MAVROS Signal-Slot Functions #######
    def mode_status_sitl(self, data):
        status = Communicate()
        status.mode_sitl_signal.connect(self.mode_status_display_sitl)
        status.mode_sitl_signal.emit(data.mode, "1")
        status.arm_signal.connect(self.arm_status_display)
        status.arm_signal.emit(data.armed, "1")

    def VFR_HUD_sitl(self, data):
        status = Communicate()
        status.airspeed_signal.connect(self.airspeed_display)
        status.airspeed_signal.emit(data.airspeed, "1")
        status.alt_signal.connect(self.altitude_display)
        status.alt_signal.emit(data.altitude, "1")

    def waypoint_sitl(self, data):
        status = Communicate()
        status.waypoint_list_signal.connect(self.waypoint_sitl_display)
        status.waypoint_list_signal.emit(data.waypoints, data.current_seq, "1")
    
    ##################################################
    # Display information from Signal Slot functions #
    ##################################################
    def airspeed_display(self, airspeed, id):
        airspeed = str(round(airspeed, 1)) + " m/s"
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftAirspeed" + id).setStyleSheet("Color: rgb(255, 0, 0);")     
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftAirspeed" + id).setPlainText(airspeed)
        self.SummaryWindow.waypoint_plaintext_dict.get("aircraftAirspeed" + id).setPlainText(airspeed)                
        rospy.logdebug_throttle(30, "[AC {} AIRSPEED display] {}".format(int(id), airspeed))

    def altitude_display(self, altitude, id):
        altitude = str(round(altitude, 1)) + " m"
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftAltitude" + id).setPlainText(altitude)
        self.SummaryWindow.waypoint_plaintext_dict.get("aircraftAltitude" + id).setPlainText(altitude)
        rospy.logdebug_throttle(30, "[AC {} ALTITUDE display] {}".format(int(id), altitude))

    def arm_status_display(self, arm_status, id):
        if arm_status == "False" or arm_status == 0:
            self.text_to_display = "DISARMED"
        else:
            if self.CommandWindow.arm_status.get('AC' + str(id)) == None or self.CommandWindow.arm_status.get('AC' + str(id)) == "DISARMED":
                self.aircrafts_info.get("AC" + id).initial_time = self.time
            self.text_to_display = "ARMED"
        self.CommandWindow.arm_status['AC' + str(id)] = self.text_to_display
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftStatus" + id).setPlainText(self.text_to_display)        
        self.SummaryWindow.waypoint_plaintext_dict.get("aircraftStatus" + id).setPlainText(self.text_to_display)
    
    def quad_batt_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftQuad Battery" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} QUAD BATT display] {}".format(int(id), data))

    def fuel_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftFuel Level" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} FUEL display] {}".format(int(id), data))

    def groundspeed_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftGroundspeed" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} GNDSPEED display] {}".format(int(id), data))

    def gps_display(self, lat, lon, id):
        data = [lat, lon]
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftGPS" + id).setPlainText(str(lat) +", " + str(lon))
        rospy.logdebug_throttle(30, "[AC {} GPS display] {}".format(int(id), data))

    def mode_status_display(self, mode_status, id):
        mode = self.CommandWindow.decoder[mode_status] # Convert the integer to its mode
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftMode" + id).setPlainText(mode)
        self.SummaryWindow.waypoint_plaintext_dict.get("aircraftMode" + id).setPlainText(mode)
        rospy.logdebug_throttle(30, "[AC {} MODE display] {}".format(int(id), mode_status))
    
    def throttle_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftThrottle" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} THROTTLE display] {}".format(int(id), data))
    
    def vibe_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftVibe Status" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} VIBE display] {}".format(int(id), data))

    def vtol_display(self, data, id):
        data = str(data)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftVTOL Status" + id).setPlainText(data)
        rospy.logdebug_throttle(30, "[AC {} VTOL display] {}".format(int(id), data))

    def waypoint_display(self, data, id):
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftTarget Waypoint" + id).setPlainText(str(data))
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + id).setRange(0,10)
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + id).setValue(data)
        self.WaypointWindow.waypoint_plaintext_dict. get("aircraft" + id).setPlainText("Current WP: " + str(data) + " out of " + str(10))

    def time_display(self, AC_time, id):
        if self.text_to_display == "DISARMED":
            self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftFlying Time" + id).setPlainText("00:00:00")
        else:
            self.time = AC_time # sync the UI time to the data time
            self.time_in_seconds = self.time - self.aircrafts_info.get("AC" + id).initial_time
            minutes = str(self.time_in_seconds // 60)
            hours = str(self.time_in_seconds // 3600)
            seconds = str(self.time_in_seconds - (minutes * 60) - (hours * 3600))
            if seconds < 10:
                seconds = "0" + seconds
            if minutes < 10:
                minutes = "0" + minutes
            if hours < 10:
                hours = "0" + hours
            self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftFlying Time" + id).setPlainText(hours + ":" + minutes + ":" + seconds)

    def status_text_display(self, status_text, id):
        self.SummaryWindow.statustext.appendPlainText(status_text)
        self.aircrafts_info.get("AC" + id).statustext.appendPlainText(id + ": " + status_text)
        rospy.logdebug("[AC %d status_text] %s", int(id), status_text)

    def ondemand_display(self, data, id):
        self.SummaryWindow.statustext.appendPlainText(str(data))
        self.aircrafts_info.get("AC" + id).statustext.appendPlainText(id + ": " + str(data))
        rospy.logdebug("[AC %d on_demand] %s", int(id), data)
    
    ####### SITL Display Functions (for developer testing only) #######
    def mode_status_display_sitl(self, mode_status, id):
        mode_status = str(mode_status)
        self.aircrafts_info.get("AC" + id).waypoint_plaintext_dict.get("aircraftMode" + id).setPlainText(mode_status)
        self.SummaryWindow.waypoint_plaintext_dict.get("aircraftMode" + id).setPlainText(mode_status)
   
    def waypoint_sitl_display(self, total, sequence, id):
        total = len(total) - 1
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + id).setRange(0,total)
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + id).setValue(sequence)
        self.WaypointWindow.waypoint_plaintext_dict.get("aircraft" + id).setPlainText("Current WP: " + str(sequence) + " out of " + str(total))
        rospy.logdebug_throttle(30, "[AC {} MODE display] {}".format(int(id), "Current WP: " + str(sequence) + " out of " + str(total)))
    
    def shortcuts(self):
        '''Create keyboard short-cuts'''
        disarming = QShortcut(self._widget)
        disarming.setKey(Qt.CTRL + Qt.Key_D)
        disarming.activated.connect(self.PopupMessages.emergency_disarm)

        shutdown = QShortcut(self._widget)
        shutdown.setKey(Qt.ALT + Qt.Key_F4)
        shutdown.activated.connect(self.shutdown_plugin)

    def shutdown_plugin(self):
        '''Shutdown function'''
        # Shutdown all the checklist windows
        for i in self.aircraft_list:
            self.checklist_info.get("AC" + str(i)).shutdown()
        # Shutdown all identifiers windows
        if sum(self.CommandWindow.is_window_open):
            if self.CommandWindow.is_window_open[0]:
                self.CommandWindow.change_identifiers_dialog.close()
            if self.CommandWindow.is_window_open[1]:
                self.CommandWindow.add_identifiers_dialog.close()
            if self.CommandWindow.is_window_open[2]:
                self.CommandWindow.edit_identifiers_dialog.close()

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
    airspeed_signal = Signal(float, str)
    alt_signal = Signal(float, str)
    arm_signal = Signal(bool, str)
    batt_signal = Signal(int, str)
    fuel_signal = Signal(int, str)
    groundspeed_signal = Signal(int, str)
    gps_signal = Signal(float, float, str)    
    mode_signal = Signal(int, str)
    throttle_signal = Signal(int, str)
    vibe_signal = Signal(int, str)
    vtol_signal = Signal(int, str)
    wp_signal = Signal(int, str)
    time_signal = Signal(int, str)

    status_text_signal = Signal(str)
    ondemand_signal = Signal(str, str)
    
    # SITL specific signals
    waypoint_list_signal = Signal(list, int, str)
    mode_sitl_signal = Signal(str, str)

def color(color):
    if color == "red":
        return QColor(255,0,0)
    elif color == "black":
        return QColor(0,0,0)
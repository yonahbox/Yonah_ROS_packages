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
from .valid_id_window import ValidIdWindow

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        self.setObjectName("main_window")
        self._widget = QWidget() 
        self._widget.setMinimumSize(1280, 800)
        self._widget.setWindowTitle("Yonah RQt")
        context.add_widget(self._widget)

        # Get path to UI file and load it
        ui_file = os.path.join(rospkg.RosPack().get_path("yonah_rqt"), "resource", "second_window.ui")
        loadUi(ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number():
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))
        
        # Declare attributes
        self.saved = ""
        self.time = 0
        self.proper_shutdown= 0
        self.destination_id = 1
        self.is_refresh_tab_open = True
        self.aircraft_list = []
        self.aircrafts_info = {}
        self.checklist_info = {}
        self.aircrafts_flight_data = {}
 
        # Declare attributes for each imported class
        self.PopupMessages = PopupMessages()
        self.WaypointWindow = WaypointWindow(self.aircraft_list)
        self.SummaryWindow = SummaryWindow(self.aircraft_list)
        self.CommandWindow = CommandWindow(self.aircraft_list)

        self.CommandWindow.change_valid_ids()
        self.create_layout()
        self.shortcuts()

        # Subscriber lists
        rospy.Subscriber("ogc/from_despatcher/regular", RegularPayload, self.regular_payload)
        rospy.Subscriber("ogc/yonahtext", String, self.status_text)
        rospy.Subscriber("ogc/from_despatcher/ondemand", String, self.ondemand)
        rospy.Subscriber("ogc/files/conflict", String, self.syncthing)
        rospy.Subscriber("ogc/feedback_to_rqt", LinkMessage, self.feedback_message)

        rospy.Subscriber("mavros/statustext/recv", StatusText, self.ondemand_sitl)
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
        self.create_tab_windows([])
        self.tab.currentChanged.connect(self.tab_change)
        
        # Add all 3 layouts into the main layout
        self._widget.verticalLayout.addWidget(scroll)
        self._widget.verticalLayout2.addWidget(self.tab)
        self._widget.verticalLayout2.addWidget(self.CommandWindow)

    def create_tab_windows(self, active_aircrafts):
        '''Create layout for Summary scroll window'''        
        if active_aircrafts == []:
            self.tab = QTabWidget()
            summary_scroll = QScrollArea()
            summary_scroll.setMinimumHeight(500)
            summary_scroll.setMinimumWidth(600)
            summary_scroll.setWidgetResizable(True)
            summary_scroll.setWidget(self.SummaryWindow)
            self.tab.addTab(summary_scroll, "Summary")

        self.SummaryWindow.create_layout(active_aircrafts)
        self.CommandWindow.create_combobox(self.aircraft_list) #for checkbox, it needs to full list and not just the added aircraft
        self.WaypointWindow.create_layout(active_aircrafts)

        for i in active_aircrafts:
            self.aircrafts_info["AC" + str(i)] = AircraftInfo(i)
            self.checklist_info["AC" + str(i)] = ChecklistWindow(i)

            tab_key = "aircraft" + str(i) + " scroll"
            self.aircrafts_info[tab_key] = QScrollArea()
            self.aircrafts_info.get(tab_key).setMinimumHeight(500)
            self.aircrafts_info.get(tab_key).setMinimumWidth(600)
            self.aircrafts_info.get(tab_key).setWidgetResizable(True)
            self.aircrafts_info.get(tab_key).setWidget(self.aircrafts_info.get("AC" + str(i)))
            self.tab.addTab(self.aircrafts_info.get(tab_key), "Aircraft " + str(i))
            self.tab.show()

        if self.tab.count() == 1:
            refresh = QScrollArea()
            self.tab.addTab(refresh, "Refresh")
        self.tab.setMinimumHeight(500)

    def tab_change(self, i):
        '''Changes the command_window drop-down menu to follow the change in tab'''
        if self.is_refresh_tab_open:
            self.tab.removeTab(1)
            self.is_refresh_tab_open = False
        active_aircrafts = self.CommandWindow.ValidIdWindow.valid_ids
        print("Activ acs: " + str(active_aircrafts))
        diff_active = list(set(active_aircrafts) - set(self.aircraft_list))
        print("diff_active: " + str(diff_active))
        if not diff_active == []:
            self.aircraft_list = active_aircrafts
            self.create_tab_windows(diff_active)
        if i == 0: # When Tab is at Summary Page, show AC 1 in the Command Window combo_box
            i = 1
        self.CommandWindow.combo_box.setCurrentIndex(i-1)

    def feedback_message(self, data):
        status = Communicate()
        status.feedback_message_signal.connect(self.feedback_message_display)
        status.feedback_message_signal.emit(data.data)

    def feedback_message_display(self, data):
        self.PopupMessages.warning_message("Command failed to send", data)

    ################################
    # Create Signal Slot functions #
    ################################
    def regular_payload(self, data):
        aircraft_id = str(data.vehicle_no)
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
        status.wp_signal.emit(data.wp, data.wp_total,aircraft_id)
        status.time_signal.connect(self.time_display)
        status.time_signal.emit(data.header.stamp.secs, aircraft_id)

    def ondemand(self, data):
        rospy.loginfo(data)
        # data. data list is i 1 1 0 message
        data_list = data.data.split()
        msg = " ".join(data_list[4:-1])
        aircraft_id = data_list[2]
        status = Communicate()
        if "LinkSwitch" in msg:
            # link_status = Communicate()
            status.ondemand_signal.connect(self.link_status)
            # link_status.ondemand_signal.emit(msg, aircraft_id)
        else:
            status.ondemand_signal.connect(self.ondemand_display)
        status.ondemand_signal.emit(msg, aircraft_id) # Change the id where to display using headers module

    def ondemand_sitl(self, data):
        status = Communicate()
        status.ondemand_signal.connect(self.ondemand_display)
        status.ondemand_signal.emit(data.text, "1")

    def status_text(self, data):
        status = Communicate()
        status.status_text_signal.connect(self.status_text_display)
        status.status_text_signal.emit(data.data)
    
    def syncthing(self, data):
        status = Communicate()
        status.syncthing_signal.connect(self.syncthing_conflict)
        status.syncthing_signal.emit(data)

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
    def airspeed_display(self, airspeed, aircraft_id):
        self.aircrafts_flight_data['airspeed' + aircraft_id] = airspeed
        airspeed = str(round(airspeed, 1)) + " m/s"
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftAirspeed" + aircraft_id).setStyleSheet("Color: rgb(255, 0, 0);")     
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftAirspeed" + aircraft_id).setPlainText(airspeed)
        self.SummaryWindow.summary_plaintext_dict.get("aircraftAirspeed" + aircraft_id).setPlainText(airspeed)

    def altitude_display(self, altitude, aircraft_id):
        self.aircrafts_flight_data['altitude' + aircraft_id] = altitude
        altitude = str(round(altitude, 1)) + " m"
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftAltitude" + aircraft_id).setPlainText(altitude)
        self.SummaryWindow.summary_plaintext_dict.get("aircraftAltitude" + aircraft_id).setPlainText(altitude)

    def arm_status_display(self, arm_status, aircraft_id):
        self.aircrafts_flight_data['status' + aircraft_id] = arm_status
        if arm_status == "False" or arm_status == 0:
            self.text_to_display = "DISARMED"
        else:
            if self.CommandWindow.arm_status.get('AC' + str(aircraft_id)) == None or self.CommandWindow.arm_status.get('AC' + str(aircraft_id)) == "DISARMED":
                self.aircrafts_info.get("AC" + aircraft_id).initial_time = self.time
            self.text_to_display = "ARMED"
        self.CommandWindow.arm_status['AC' + str(aircraft_id)] = self.text_to_display
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftStatus" + aircraft_id).setPlainText(self.text_to_display)        
        self.SummaryWindow.summary_plaintext_dict.get("aircraftStatus" + aircraft_id).setPlainText(self.text_to_display)
    
    def quad_batt_display(self, data, aircraft_id):
        self.aircrafts_flight_data['battery' + aircraft_id] = data
        data = str(data)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftQuad Battery" + aircraft_id).setPlainText(data)

    def fuel_display(self, data, aircraft_id):
        self.aircrafts_flight_data['fuel' + aircraft_id] = data
        data = str(data)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftFuel Level" + aircraft_id).setPlainText(data)

    def groundspeed_display(self, gndspeed, aircraft_id):
        self.aircrafts_flight_data['groundspeed' + aircraft_id] = gndspeed
        data = str(round(gndspeed, 1)) + " m/s"
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftGroundspeed" + aircraft_id).setPlainText(data)

    def gps_display(self, lat, lon, aircraft_id):
        data = [lat, lon]
        self.aircrafts_flight_data['gps' + aircraft_id] = data
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftGPS" + aircraft_id).setPlainText(str(lat) +", " + str(lon))

    def mode_status_display(self, mode_status, aircraft_id):
        self.aircrafts_flight_data['mode' + aircraft_id] = mode_status
        mode = self.CommandWindow.decoder[mode_status] # Convert the integer to its mode
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftMode" + aircraft_id).setPlainText(mode)
        self.SummaryWindow.summary_plaintext_dict.get("aircraftMode" + aircraft_id).setPlainText(mode)
    
    def throttle_display(self, data, aircraft_id):
        self.aircrafts_flight_data['throttle' + aircraft_id] = data
        data = str(data)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftThrottle" + aircraft_id).setPlainText(data)
    
    def vibe_display(self, data, aircraft_id):
        self.aircrafts_flight_data['vibe' + aircraft_id] = data
        data = str(data)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftVibe Status" + aircraft_id).setPlainText(data)

    def vtol_display(self, data, aircraft_id):
        self.aircrafts_flight_data['vtol' + aircraft_id] = data
        data = str(data)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftVTOL Status" + aircraft_id).setPlainText(data)

    def waypoint_display(self, waypoint, total_waypoint, aircraft_id):
        self.aircrafts_flight_data['waypoint' + aircraft_id] = (waypoint, total_waypoint)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftTarget Waypoint" + aircraft_id).setPlainText(str(waypoint))
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + aircraft_id).setRange(0,total_waypoint)
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + aircraft_id).setValue(waypoint)
        self.WaypointWindow.waypoint_plaintext_dict. get("aircraft" + aircraft_id).setPlainText("Current WP: " + str(waypoint) + " out of " + str(total_waypoint))

    def time_display(self, AC_time, aircraft_id):
        if self.text_to_display == "DISARMED":
            self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftFlying Time" + aircraft_id).setPlainText("00:00:00")
        else:
            self.time = AC_time # sync the UI time to the data time
            time_in_seconds = int(self.time) - int(self.aircrafts_info.get("AC" + aircraft_id).initial_time)
            minutes = str(time_in_seconds // 60)
            hours = str(time_in_seconds // 3600)
            seconds = str(time_in_seconds - (int(minutes) * 60) - (int(hours) * 3600))
            if int(seconds) < 10:
                seconds = "0" + seconds
            if int(minutes) < 10:
                minutes = "0" + minutes
            if int(hours) < 10:
                hours = "0" + hours
            self.aircrafts_flight_data['time' + aircraft_id] = self.aircrafts_info.get("AC" + aircraft_id).initial_time
            self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftFlying Time" + aircraft_id).setPlainText(hours + ":" + minutes + ":" + seconds)

    def link_status(self, link, aircraft_id):
        link = link[-1] # extract the status
        rospy.loginfo(link)
        if int(link) == 0:
            link = "Telegram"
        elif int(link) == 1:
            link = "SMS"
        elif int(link) == 2:
            link = "SBD"
        else:
            link = "ERR"
        self.WaypointWindow.waypoint_plaintext_dict.get("aircraftlink" + aircraft_id).setPlainText(link)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftLink Status" + aircraft_id).setPlainText(link)
        self.SummaryWindow.summary_plaintext_dict.get("aircraftLink Status" + aircraft_id).setPlainText(link)

    def status_text_display(self, status_text):
        status = status_text.split(",")
        # time_stamp = int(status[-1]) # Timestamp not needed
        message_type = status[0]
        aircraft_id = status[2]
        
        info = status[4:-1]
        display_text = "Aircraft {} : {}".format(aircraft_id, str(info[0]))
        self.SummaryWindow.statustext.appendPlainText(display_text)
        self.aircrafts_info.get("AC" + aircraft_id).statustext.appendPlainText(display_text)

    def ondemand_display(self, data, aircraft_id):
        status = ""
        text_to_display = "Aircraft {} {}: {}".format(aircraft_id, status, data)
        self.SummaryWindow.statustext.appendPlainText(text_to_display)
        self.aircrafts_info.get("AC" + aircraft_id).statustext.appendPlainText(text_to_display)
    
    ####### SITL Display Functions (for developer testing only) #######
    def mode_status_display_sitl(self, mode_status, aircraft_id):
        self.aircrafts_flight_data['mode_sitl' + aircraft_id] = mode_status
        mode_status = str(mode_status)
        self.aircrafts_info.get("AC" + aircraft_id).aircraft_info_dict.get("aircraftMode" + aircraft_id).setPlainText(mode_status)
        self.SummaryWindow.summary_plaintext_dict.get("aircraftMode" + aircraft_id).setPlainText(mode_status)
        self.saved =  "[AC {} MODE display] {}".format(int(aircraft_id), mode_status)
   
    def waypoint_sitl_display(self, total, sequence, aircraft_id):
        self.aircrafts_flight_data['waypoint_sitl' + aircraft_id] = [total, sequence]
        total = len(total) - 1
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + aircraft_id).setRange(0,total)
        self.WaypointWindow.waypoint_plaintext_dict.get("progress_bar_aircraft" + aircraft_id).setValue(sequence)
        self.WaypointWindow.waypoint_plaintext_dict.get("aircraft" + aircraft_id).setPlainText("Current WP: " + str(sequence) + " out of " + str(total))
    
    def shortcuts(self):
        '''Create keyboard short-cuts'''
        disarming = QShortcut(self._widget)
        disarming.setKey(Qt.CTRL + Qt.Key_D)
        disarming.activated.connect(self.PopupMessages.emergency_disarm)

        shutdown = QShortcut(self._widget)
        shutdown.setKey(Qt.ALT + Qt.Key_F4)
        shutdown.activated.connect(self.shutdown_plugin)

    def syncthing_conflict(self, file_name):
        heading = "Your edit to the file {} encountered an error".format(file_name)
        info_text = "This might be caused due edits happening at the same time \n Please check whether there is another person editing the same file"
        self.PopupMessages.warning_message(heading, info_text)

    def shutdown_plugin(self):
        self.proper_shutdown = 1
        '''Shutdown function'''
        # Shutdown all the checklist windows
        for i in self.aircraft_list:
            self.checklist_info.get("AC" + str(i)).shutdown()
        # Shutdown all identifiers windows
        opened_CommandWindows = [x for x in self.CommandWindow.windows_opened.keys() if self.CommandWindow.windows_opened.get(x)]
        opened_PopupMessages = [x for x in self.PopupMessages.windows_opened.keys() if self.PopupMessages.windows_opened.get(x)]
        opened_windows = opened_CommandWindows + opened_PopupMessages
        for i in opened_windows:
            if i == "full_menu":
                self.CommandWindow.full_widget.close()
            elif i == "change_identifiers_dialog":
                self.CommandWindow.change_identifiers_dialog.close()
            elif i == "add_identifiers_dialog":
                self.CommandWindow.add_identifiers_dialog.close()
            elif i == "edit_identifiers_dialog":
                self.CommandWindow.edit_identifiers_dialog.close()
            elif i == "arm window" or i == "disarm window":
                self.CommandWindow.PopupMessages.message.close()
            elif i == "checklist window":
                self.CommandWindow.checklist_info.get("AC" + str(self.destination_id)).close()
            elif i == "change_valid_ids":
                self.CommandWindow.ValidIdWindow.close()

        self.WaypointWindow.shutdown()
        self.SummaryWindow.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        with open(self.path, 'r') as lines:
            data = lines.readlines()
        for i in range (len(data)):
            data[i] = "None\n"
        with open(self.path, 'w') as files:
            files.writelines(data)
            files.close()

    def restore_settings(self, plugin_settings, instance_settings):
        filename = "rqt_log.txt"
        self.path = os.path.join(rospkg.RosPack().get_path("yonah_rqt"), "src/yonah_rqt", filename)
        file_exists = os.path.isfile(self.path) 
        if file_exists:
            with open(self.path, 'r') as lines:
                data = lines.readlines()
            for i in data:
                i = i[:-1]
                if i == "None":
                    continue
                else:
                    regpay = i.split(" ")
                    msg = regular.convert_to_rosmsg(regpay)
                    self.regular_payload(msg)
        else:
            rospy.loginfo("rqt_log file doesn't exist, creating new rqt_log file")
            f = open(self.path, "w+")
            f.write("None")
   
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
    wp_signal = Signal(int, int, str)
    time_signal = Signal(int, str)

    status_text_signal = Signal(str)
    ondemand_signal = Signal(str, str)
    syncthing_signal = Signal(str)

    feedback_message_signal = Signal(str)
    # SITL specific signals
    waypoint_list_signal = Signal(list, int, str)
    mode_sitl_signal = Signal(str, str)

def color(color):
    if color == "red":
        return QColor(255,0,0)
    elif color == "black":
        return QColor(0,0,0)

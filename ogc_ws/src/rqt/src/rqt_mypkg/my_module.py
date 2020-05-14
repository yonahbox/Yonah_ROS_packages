#!/usr/bin/env python

import os
import rospy
import rospkg
import rosservice
import rostopic

from std_msgs.msg import String
from mavros_msgs.msg import StatusText, State, VFR_HUD, WaypointReached, WaypointList
from sensor_msgs.msg import NavSatFix
#from despatcher.msg import RegularPayload
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot, QAbstractListModel, QObject
from python_qt_binding.QtGui import QIcon, QImage, QPainter
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter
from python_qt_binding.QtSvg import QSvgGenerator

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        self._widget = QWidget()
        if context.serial_number():
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        ###############################################
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        # parser.add_argument("-q", "--quiet", action="store_true",
        #               dest="quiet",
        #               help="Put plugin in silent mode")
        # args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     print('arguments: ', args)
        #     print('unknowns: ', unknowns)
        ###############################################

        # Create QWidget
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi') #[DK] what does this line do?
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        self.waypoint_list = [0,0]

        self._widget.arming_pushbutton.pressed.connect(self.arming)
        self._widget.control_pushbutton.pressed.connect(self.transfer_control)
        self._widget.mode_manual_pushbutton.pressed.connect(self.mode_manual)
        self._widget.mode_auto_pushbutton.pressed.connect(self.mode_auto)
        self._widget.load_mission_pushbutton.pressed.connect(self.load_mission)
        self._widget.check_mission_pushbutton.pressed.connect(self.check_mission)
        self._widget.checklist_pushbutton.pressed.connect(self.checklist)
        
        #Subscriber lists
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.status_text)
        rospy.Subscriber("mavros/state", State, self.mode_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.VFR_HUD)
        #rospy.Subscriber("mavros/mission/reached", WaypointReached, self.waypoint_index)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoint_total)
        #rospy.Subscriber("mavros/global_position/global", NavSatFix, self.GPS_data)
        #rospy.Subscriber('ogc/from_despatcher/regular', RegularPayload, self.despatcher_message)

        context.add_widget(self._widget)


    def status_text(self, data):
        status = Communicate()
        status.status_text_signal.connect(self.status_text_display)
        status.status_text_signal.emit(data.text)
    
    def mode_status(self, data):
        status = Communicate()
        status.mode_signal.connect(self.mode_status_display)
        status.mode_signal.emit(data.mode)
        status.arm_signal.connect(self.arm_status_display)
        status.arm_signal.emit(data.armed)

    def VFR_HUD(self, data):
        status = Communicate()
        status.airspeed_signal.connect(self.airspeed_display)
        status.airspeed_signal.emit(data.airspeed)
        status.altitude_signal.connect(self.altitude_display)
        status.altitude_signal.emit(data.altitude)
    
    # def GPS_data(self, data):
    #     status = Communicate()
    #     status.gps_signal.connect(self.altitude_display)
    #     status.gps_signal.emit(data.altitude)

    # def waypoint_index(self, data):
    #     status = Communicate()
    #     status.waypoint_index_signal.connect(self.waypoint_index_display)
    #     status.waypoint_index_signal.emit(data.wp_seq)  

    def waypoint_total(self, data):
        status = Communicate()
        status.waypoint_list_signal.connect(self.waypoint_total_display)
        status.waypoint_list_signal.emit(data.waypoints, data.current_seq)

    def status_text_display(self, status_text):
        self._widget.message_textedit.append(status_text)
    
    def mode_status_display(self, mode_status):
        self._widget.mode_textedit.setText(mode_status)

    def altitude_display(self, altitude):
        altitude = str(round(altitude, 1)) + ' m'
        self._widget.altitude_textedit.setText(altitude)

    def airspeed_display(self, airspeed):
        airspeed = str(round(airspeed, 1)) + ' m/s'
        self._widget.airspeed_textedit.setText(airspeed)

    def waypoint_total_display(self, total, sequence):
        print('total wp list: ' + str(total))
        print('current wp: ' + str(sequence))
        print('------------------------------------')
        total = len(total) - 1
        
        self._widget.progressBar.setRange(0,total)
        self._widget.progressBar.setValue(sequence)
        self._widget.wp_textedit.setText('Current WP: ' + str(sequence) + ' out of ' + str(total))


    # def waypoint_index_display(self, index):
    #     print ('index:::: ' + str(index)
    #     self.waypoint_list[1] = index
        

    # def waypoint_display(self, waypoint):
        
    #     self._widget.wp_textedit.setText(str(waypoint+1))

    def arm_status_display(self, arm_status):
        if arm_status == False:
            text_to_display = 'DISARMED'
        else:
            text_to_display = 'ARMED'
        self._widget.arming_textedit.setText(str(text_to_display))

    # def despatcher_message(self,data):
    #     return 0

    def arming (self):
        print ('arming pushbutton successful')
        print ('a')
        
    def transfer_control (self):
        print ('transfer control successful')
    def mode_manual (self):
        print ('manual mode activated')
    def mode_auto (self):
        print ('auto mode activated')
    def load_mission(self):
        print ('mission loaded')
    def check_mission(self):
        print ('checked mission')
    def checklist(self):
        print ('BTO BPO checklist')
    def armingbox(self):
        print ('successful!')

    
    def shutdown_plugin(self):
        self.sub.unregister()
        pass

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

   
class Communicate (QObject):
    #technically, all the signals that has the same input, such as string
    #can be combined into one variable. However, for clarity purpose,
    #I split each variable for each display we need to make
    status_text_signal = Signal(str)
    arm_signal = Signal(bool)
    mode_signal = Signal(str)
    altitude_signal = Signal(float)
    airspeed_signal = Signal(float)
    waypoint_list_signal = Signal(list, int)
    #waypoint_index_signal = Signal(int)

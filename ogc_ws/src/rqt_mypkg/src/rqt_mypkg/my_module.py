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
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QScrollArea, QPushButton, QVBoxLayout, QCheckBox
from python_qt_binding.QtSvg import QSvgGenerator
from checklist_window import ChecklistWindow

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
        #[DA] os.path.join is a method that joins paths, it concatenates various path components with a
        #[DA] directory separator (/) sollowing each non-empty part except the last component
        #[DA] the code means get directory to the folder 'rqt_mypkg' and then navigate to resource then access main_window.ui
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'main_window.ui')
        
        # Extend the widget with all attributes and children from UI file
        # there is actually a third param available if we have custom class in our widget, but I think we can ignore this
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MainWindowUI') #[DA] This sets the name instance using property inherited by _widget
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number():
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        
        self.ChecklistWindow = ChecklistWindow()
        self.checklist_opened = 0
        self._widget.arming_pushbutton.pressed.connect(self.arming)
        self._widget.control_pushbutton.pressed.connect(self.transfer_control)
        self._widget.mode_manual_pushbutton.pressed.connect(self.mode_manual)
        self._widget.mode_auto_pushbutton.pressed.connect(self.mode_auto)
        self._widget.load_mission_pushbutton.pressed.connect(self.load_mission)
        self._widget.check_mission_pushbutton.pressed.connect(self.check_mission)
        self._widget.checklist_pushbutton.pressed.connect(self.ChecklistWindow.show)
        
        #Subscriber lists
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.status_text)
        rospy.Subscriber("mavros/state", State, self.mode_status)
        rospy.Subscriber("mavros/vfr_hud", VFR_HUD, self.VFR_HUD)
        #rospy.Subscriber("mavros/mission/reached", WaypointReached, self.waypoint_index)
        rospy.Subscriber("mavros/mission/waypoints", WaypointList, self.waypoint_total)
        #rospy.Subscriber("mavros/global_position/global", NavSatFix, self.GPS_data)
        #rospy.Subscriber('ogc/from_despatcher/regular', RegularPayload, self.despatcher_message)

        # Publisher List
        self.arming_publisher = rospy.Publisher('send_arming', String, queue_size = 5)
        self.transfer_control_publisher = rospy.Publisher('transfer_control', String, queue_size = 5)
        self.mode_publisher = rospy.Publisher('mode_change', String, queue_size = 5)
        self.mission_publisher = rospy.Publisher('load_mission', String, queue_size = 5)
        self.rate = rospy.Rate(2)
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
    
    # All functions with _display are the functions that takes the information and display it to the UI
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
        total = len(total) - 1
        self._widget.progressBar.setRange(0,total)
        self._widget.progressBar.setValue(sequence)
        self._widget.wp_textedit.setText('Current WP: ' + str(sequence) + ' out of ' + str(total))

    def arm_status_display(self, arm_status):
        if arm_status == False:
            self.text_to_display = 'DISARMED'
        else:
            self.text_to_display = 'ARMED'
        self._widget.arming_textedit.setText(str(self.text_to_display))

    def arming (self):
        print(self.checklist_opened)
        self.arming_publisher.publish('ARM')
        self.status_text_display('Arming message sent')
        # if self.text_to_display == 'DISARMED':
        #     self.arming.publish('ARM')
        # elif self.text_to_display == 'ARMED':
        #     self.arming.publish('DISARM')
        #     print('Now disarm it')

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
    def armingbox(self):
        print ('successful!')
    
    def shutdown_plugin(self):
        self.ChecklistWindow.shutdown()

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
    # technically, all the signals that has the same input, such as string
    # can be combined into one variable. However, for clarity purpose,
    # I split each variable for each display we need to make
    status_text_signal = Signal(str)
    arm_signal = Signal(bool)
    mode_signal = Signal(str)
    altitude_signal = Signal(float)
    airspeed_signal = Signal(float)
    waypoint_list_signal = Signal(list, int)
    waypoint_index_signal = Signal(int)



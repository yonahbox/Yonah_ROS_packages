import os
import rospkg
import __main__

from python_qt_binding import loadUi
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from checklist_window import ChecklistWindow

### File is still changing rapidly and dynamically, hence comments might not be accurate
class CommandWindow(QWidget):
    def __init__(self, active_aircrafts):
        super(CommandWindow, self).__init__()
        self.setWindowTitle("Command Window")
        
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.first_row = QHBoxLayout()
        self.second_row = QHBoxLayout()
        self.setLayout(self.main_layout) # Set main_layout as the layout that occupies the entire widget

        # Create the widgets
        self.combo_box = QComboBox()
        for i in range(1, active_aircrafts + 1): # Use a for loop to add items inside the drop down menu
            self.combo_box.addItem('Aircraft ' + str(i))
        self.arm_button = QPushButton('ARM / DISARM')
        self.go_button = QPushButton('GO / RETURN')
        self.checklist_button = QPushButton('Checklist')
        self.mission_load_button = QPushButton('Load Mission')
        self.mission_check_button = QPushButton('Check Mission')

        # set UI properties of the buttons and layout
        self.first_row.setContentsMargins(0,20,0,20)
        self.arm_button.setMinimumHeight(50)
        self.go_button.setMinimumHeight(50)
        self.mission_check_button.setMinimumHeight(30)
        self.mission_load_button.setMinimumHeight(30)
        self.checklist_button.setMinimumHeight(30)
       
        # add the widgets into the layouts
        self.main_layout.addWidget(self.combo_box)
        self.first_row.addWidget(self.arm_button)
        self.first_row.addWidget(self.go_button)
        self.second_row.addWidget(self.checklist_button)
        self.second_row.addWidget(self.mission_load_button)
        self.second_row.addWidget(self.mission_check_button)
        self.main_layout.addLayout(self.first_row)
        self.main_layout.addLayout(self.second_row)

    def shutdown(self):
        self.close()
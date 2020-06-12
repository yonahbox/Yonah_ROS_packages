import os
import rospkg
import __main__

from python_qt_binding import loadUi
from PyQt5.QtWidgets import *
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
from checklist_window import ChecklistWindow

### File is still changing rapidly and dynamically, hence comments might not be accurate
class CommandWindow(QWidget):
    def __init__(self):
        super(CommandWindow, self).__init__()
        self.setWindowTitle("Command Window")
        
        # create the layout
        self.layout = QVBoxLayout(self)
        self.first_row = QHBoxLayout(self)
        self.second_row = QHBoxLayout(self)
        self.first_row.setContentsMargins(0,20,0,20)

        # create the widgets
        self.combo_box = QComboBox()
        self.active_aircrafts = 5
        for i in range(1, self.active_aircrafts):
            self.combo_box.addItem('Aircraft ' + str(i))
        self.arm_button = QPushButton('ARM / DISARM')
        self.go_button = QPushButton('GO / RETURN')
        self.checklist_button = QPushButton('Checklist')
        self.mission_load_button = QPushButton('Load Mission')
        self.mission_check_button = QPushButton('Check Mission')

        self.arm_button.setMinimumHeight(50)
        self.go_button.setMinimumHeight(50)
        self.mission_check_button.setMinimumHeight(30)
        self.mission_load_button.setMinimumHeight(30)
        self.checklist_button.setMinimumHeight(30)
        

        
        # add the widgets into the layouts
        self.layout.addWidget(self.combo_box)
        self.first_row.addWidget(self.arm_button)
        self.first_row.addWidget(self.go_button)
        self.second_row.addWidget(self.checklist_button)
        self.second_row.addWidget(self.mission_load_button)
        self.second_row.addWidget(self.mission_check_button)
        self.layout.addLayout(self.first_row)
        self.layout.addLayout(self.second_row)
        self.setLayout(self.layout)


    def shutdown(self):
        self.close()
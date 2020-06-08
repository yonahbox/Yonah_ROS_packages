#!/usr/bin/env python3
import os
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QLabel
from python_qt_binding.QtWidgets import QScrollArea, QPushButton, QVBoxLayout, QCheckBox, QHBoxLayout
from python_qt_binding.QtWidgets import QAction, QTreeWidget, QTreeWidgetItem, QMessageBox
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot
import __main__

class ControlWindow(QWidget):
    def __init__(self):
        super(ControlWindow, self).__init__()
        self.setWindowTitle("Ground Control Station")
        self.resize(500, 700)
        self.move(200,100)
        
        # create the layout
        self.layout = QVBoxLayout(self)
        self.buttons_layout = QHBoxLayout(self)
        self.tree_widget_layout = QHBoxLayout(self)
        
        # create the widgets
        self.create_widget()
        self.has_message_opened = 0

        # add the widgets into the layouts
        self.layout.addLayout(self.tree_widget_layout)
        self.layout.addLayout(self.buttons_layout)

    def create_widget(self):
        # declare buttons and connect each of them to a function
        self.load_button = QPushButton('Load')
        self.ok_button = QPushButton('OK')
        self.cancel_button = QPushButton('Cancel')
        
        
        self.buttons_layout.addWidget(self.load_button)
        self.buttons_layout.addWidget(self.cancel_button)
        self.buttons_layout.addWidget(self.ok_button)
        self.setLayout(self.layout)

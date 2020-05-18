import os
import rospy
import rospkg
import rosservice
import rostopic
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QScrollArea, QPushButton, QVBoxLayout, QCheckBox
from PyQt5 import QtCore, QtGui, QtWidgets

class NewWindow(QWidget):
    def __init__(self):
        super(NewWindow, self).__init__()
        self.setWindowTitle("BTO and BPO Checlist")
        self.resize(500, 700)
        print('newWindow is activated')     
        self.area = QScrollArea(self)
        self.main_widget = QWidget(self.area)
        self.ok_button = QPushButton("Record", self)
        #self.ok_button.clicked.connect(self.onButtonClicked)
        self.ok_button.setEnabled(False)

        self.from_nodes_button = QPushButton("From Nodes", self)
        #self.from_nodes_button.clicked.connect(self.onFromNodesButtonClicked)

        self.main_vlayout = QVBoxLayout(self)
        self.main_vlayout.addWidget(self.area)
        self.main_vlayout.addWidget(self.ok_button)
        self.main_vlayout.addWidget(self.from_nodes_button)
        self.setLayout(self.main_vlayout)

        self.selection_vlayout = QVBoxLayout(self.main_widget)
        self.item_all = QCheckBox("All", self)
        self.testing = QCheckBox("Not all", self)
        self.item_all.stateChanged.connect(self.bto_checklist)
        self.selection_vlayout.addWidget(self.item_all)
        #topic_data_list = master.getPublishedTopics('')
        # topic_data_list.sort()
        # for topic, datatype in topic_data_list:
        #     self.addCheckBox(topic)
        self.main_widget.setLayout(self.selection_vlayout)
        self.area.setWidget(self.main_widget)
        print('show is activating')
        print('show is activated')

    def bto_checklist(self, state):
        pass
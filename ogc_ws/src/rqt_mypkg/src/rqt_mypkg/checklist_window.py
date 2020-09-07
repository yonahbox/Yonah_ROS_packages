#!/usr/bin/env python3
'''
checklist_window: Additional window that contains the BPO and BTO checklists

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
import csv
import os
import rospkg
from python_qt_binding.QtWidgets import QFileDialog, QWidget, QPushButton, QVBoxLayout
from python_qt_binding.QtWidgets import QHBoxLayout, QTreeWidget, QTreeWidgetItem, QMessageBox
from python_qt_binding.QtCore import Qt

class ChecklistWindow(QWidget):
    def __init__(self, aircraft_id):
        super(ChecklistWindow, self).__init__()
        # Set properties of the window
        self.setWindowTitle("BTO and BPO Checklist")
        self.resize(500, 700)
        self.move(200,100)
        self.aircraft_id = aircraft_id
        self.checklist_state = 0

        # Relative path for the default BPO and BTO checklist
        BPO_checklist_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'src/rqt_mypkg', 'BPO_checklist.csv')
        BTO_checklist_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'src/rqt_mypkg', 'BTO_checklist.csv')
        
        # Check whether checklist is present, if not print a error message to terminal
        try:
            # If checklist is present, parse it and pass it to its respective variable
            self.BPO_checklist = self.excel_parser(BPO_checklist_file)
            self.BTO_checklist = self.excel_parser(BTO_checklist_file)
        except:
            print("\033[91m ERROR: Checklist files are missing or named wrongly. Please follow the original directory and naming")
            exit()
        
        # Create the layout
        self.main_layout = QVBoxLayout()
        self.buttons_layout = QHBoxLayout()
        self.tree_widget_layout = QHBoxLayout()
        
        # Create the widgets
        self.create_widget()
        self.has_message_opened = 0

        # Add the widgets into the layouts
        self.main_layout.addLayout(self.tree_widget_layout)
        self.main_layout.addLayout(self.buttons_layout)
        self.setLayout(self.main_layout)

    # Create the main layout of widget
    def create_widget(self):
        # Create tree structure
        self.create_tree()

        # Declare buttons and connect each of them to a function
        self.load_button = QPushButton('Load')
        self.ok_button = QPushButton('OK')
        self.cancel_button = QPushButton('Cancel')
        self.load_button.pressed.connect(self.load_clicked)
        self.ok_button.pressed.connect(self.ok_clicked)
        self.cancel_button.pressed.connect(self.cancel_clicked)
        
        # Add buttons into the layout
        self.buttons_layout.addWidget(self.load_button)
        self.buttons_layout.addWidget(self.cancel_button)
        self.buttons_layout.addWidget(self.ok_button)
        self.tree_widget_layout.addWidget(self.tree_widget)

    # Create the tree layout of widget inside of the main layout
    def create_tree(self):
        # Set up the main tree widget
        self.tree_widget = QTreeWidget()
        self.tree_widget.setColumnCount(2)
        self.tree_widget.setColumnWidth(0, 250)
        self.tree_widget.setHeaderLabels(['Parts', 'Status'])
        self.item = QTreeWidgetItem()
        
        # Create the BPO section
        self.BPO_header = QTreeWidgetItem(self.tree_widget)
        self.BPO_header.setFlags(self.BPO_header.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self.BPO_header.setText(0, 'BPO Checklist')
        self.BPO_header.setExpanded(True)
        self.create_item(self.BPO_header, self.BPO_checklist) # Adds the list of items into the section

        # Create the BTO section
        self.BTO_header = QTreeWidgetItem(self.tree_widget)
        self.BTO_header.setFlags(self.BTO_header.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self.BTO_header.setText(0, 'BTO Checklist')
        self.BTO_header.setExpanded(True)
        self.create_item(self.BTO_header, self.BTO_checklist) # Adds the list of items into the section

    # Populate the tree layout with items
    def create_item(self, parent, list):
        section_header = [] # List of the section headers
        for i in range (len(list)):
            if (list[i][1] == '' and list[i][0] != ''):
                section_header.append(list[i][0])
        k = 0

        # Iterate through the different sections
        for j in range (len(section_header)):
            # Child refers to the sections (mechanical, avionics, etc)
            child = QTreeWidgetItem(parent)
            child.setFlags(child.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
            child.setText(0, section_header[j])

            while k < len(list):
                if list[k][0] in section_header:
                    # When the while loop encounters the first title, continue the loop
                    if (list[k][0] == section_header[0]):
                        k += 1
                        continue
                    # When the while loop encounters the next titles, break the loop so that the value of j increases
                    k += 1 
                    break
                # when the list contains empty cells, skip the cell and continue
                elif list[k][0] == '':
                    k += 1
                    continue
                # Add the list items to the treewidgetitem
                # Grandchild refers to the items under each section (wing nuts, tail nuts, etc)
                grandchild = QTreeWidgetItem(child)
                grandchild.setText(0, list[k][0])
                grandchild.setText(1, list[k][1])
                grandchild.setCheckState(0, Qt.Unchecked) # Set all checkbox to its unchecked state
                k += 1

    # Read the excel sheet and parse it as an array
    def excel_parser(self, file_name):
        with open(file_name, 'r') as file:
            checklist = []
            reader = csv.reader(file)
            for row in reader:
                checklist.append(row)
            return checklist            
    
    # Determines what happens when load button is clicked
    def load_clicked(self):
        # Use QFileDialog to open the system's file browser
        filenames = QFileDialog.getOpenFileNames(
            self, self.tr('Load from Files'), '.', self.tr('csv files {.csv} (*.csv)'))
        # Iterate through the file names selected
        for filename in filenames[0]:
            # If the file names has the word BPO or BTO in it, remove current widget, add the loaded one
            if (filename.find('BPO') != -1):
                self.BPO_checklist = self.excel_parser(filename)
                self.remove_widget()
                self.create_widget()
            elif (filename.find('BTO') != -1):
                self.BTO_checklist = self.excel_parser(filename)
                self.remove_widget()
                self.create_widget()
            else:
                print('ERROR: Checklist name must contain BPO or BTO')
                self.close()
    
    # Close all the main_layout
    def remove_widget(self):
        self.main_layout.removeWidget(self.tree_widget)
        self.tree_widget.deleteLater()
        self.buttons_layout.removeWidget(self.ok_button)
        self.buttons_layout.removeWidget(self.cancel_button)
        self.buttons_layout.removeWidget(self.load_button)
        self.ok_button.deleteLater()
        self.cancel_button.deleteLater()
        self.load_button.deleteLater()
    
     # Declare what will happen when ok button is clicked
    def ok_clicked(self):
        if self.BPO_header.checkState(0) != 2 or self.BTO_header.checkState(0) != 2: # User clicks ok without checking all
            self.dialog_window("Some items in the checklist are still unchecked", "Do you still want to continue?", True)
        else:
            self.checklist_state = 1
            self.close()

    # Declare what will happen when cancel button is clicked
    def cancel_clicked(self):
        if self.BPO_header.checkState(0) != 0 or self.BTO_header.checkState(0) != 0: # User clicks cancel with some boxes checked
            self.dialog_window('Some of your items are checked. Cancelling will uncheck all your items', 'Do you still want to continue?', False)
        else:
            self.BTO_header.setCheckState(0, Qt.Unchecked)
            self.BPO_header.setCheckState(0, Qt.Unchecked)
            self.close()
    
    # Create a pop up window when user pre-emptively cancelled or clicked ok without completing the checklist
    def dialog_window(self, message, detail, check):
        self.message = QMessageBox()
        self.has_message_opened = 1
        self.message.setIcon(QMessageBox.Warning)
        self.message.setText(message)
        self.message.setInformativeText(detail)
        self.message.setWindowTitle("Items are unchecked")
        self.message.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        self.message.show()
        if check == True: # Check == True means it is from ok_clicked
            self.message.buttonClicked.connect(self.message_action)
        else:
            self.message.buttonClicked.connect(self.message_action_uncheck)
    
    # Determines what happens after dialog_window pops up from ok_button
    def message_action(self, i):
        if i.text() == '&Yes':
            self.checklist_state = 1
            self.close()
        else:
            self.message.close()

    # Determines what happens after dialog_window pops up from cancel_button
    def message_action_uncheck(self, i):
        self.response = i.text()
        if self.response == '&Yes':
            self.checklist_state = 1
            self.BTO_header.setCheckState(0, Qt.Unchecked)
            self.BPO_header.setCheckState(0, Qt.Unchecked)
            self.close()
        else:
            self.message.close()

    # Shutdown function
    def shutdown(self):
        self.close()
        if self.has_message_opened == 1:
            self.message.close()


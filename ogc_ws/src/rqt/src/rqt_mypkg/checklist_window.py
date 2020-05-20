# TODO streamline the import files and only include those that are really needed
import csv
from confirmation_window import ConfirmationWindow
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsScene, QWidget, QCompleter, QLabel
from python_qt_binding.QtWidgets import QScrollArea, QPushButton, QVBoxLayout, QCheckBox, QHBoxLayout
from python_qt_binding.QtWidgets import QAction, QTreeWidget, QTreeWidgetItem, QMessageBox
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot



class ChecklistWindow(QWidget):
    def __init__(self):
        super(ChecklistWindow, self).__init__()
        self.setWindowTitle("BTO and BPO Checklist")
        self.resize(500, 700)
        self.move(200,100)
        # absolute path for the default BPO and BTO checklist
        self.BPO_checklist = self.excel_parser('/home/dani/catkin_ws/src/rqt_mypkg/src/rqt_mypkg/BPO_checklist.csv')
        self.BTO_checklist = self.excel_parser('/home/dani/catkin_ws/src/rqt_mypkg/src/rqt_mypkg/BTO_checklist.csv')
        # create the layout
        self.layout = QVBoxLayout(self)
        self.buttons_layout = QHBoxLayout(self)
        self.tree_widget_layout = QHBoxLayout(self)
        
        # create the widgets
        self.create_widget()

        # add the widgets into the layouts
        self.layout.addLayout(self.tree_widget_layout)
        self.layout.addLayout(self.buttons_layout)
        
    def create_widget(self):
        self.create_tree()
        
        # declare buttons and connect each of them to a function
        self.load_button = QPushButton('Load')
        self.ok_button = QPushButton('OK')
        self.cancel_button = QPushButton('Cancel')
        self.load_button.pressed.connect(self.load_clicked)
        self.ok_button.pressed.connect(self.ok_clicked)
        self.cancel_button.pressed.connect(self.cancel_clicked)
        
        self.buttons_layout.addWidget(self.load_button)
        self.buttons_layout.addWidget(self.cancel_button)
        self.buttons_layout.addWidget(self.ok_button)
        self.tree_widget_layout.addWidget(self.tree_widget)
        self.setLayout(self.layout)

    def create_tree(self):
        print('create tree is called again')
        # set up the main tree widget
        self.tree_widget = QTreeWidget()
        self.tree_widget.setColumnCount(2)
        self.tree_widget.setColumnWidth(0, 250)
        self.tree_widget.setHeaderLabels(['Parts', 'Status'])
        self.item = QTreeWidgetItem()
        
        # create the first header
        self.BPO_header = QTreeWidgetItem(self.tree_widget)
        self.BPO_header.setFlags(self.BPO_header.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self.BPO_header.setText(0, 'BPO Checklist')
        self.BPO_header.setExpanded(True)
        self.create_item(self.BPO_header, self.BPO_checklist)

        self.BTO_header = QTreeWidgetItem(self.tree_widget)
        self.BTO_header.setFlags(self.BTO_header.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
        self.BTO_header.setText(0, 'BTO Checklist')
        self.BTO_header.setExpanded(True)
        self.create_item(self.BTO_header, self.BTO_checklist)

    def create_item(self, parent, list):
        section_header = []
        for i in range (len(list)):
            if (list[i][1] == '' and list[i][0] != ''):
                section_header.append(list[i][0])
        k = 0
        for j in range (len(section_header)):
            # child refers to the section headers (mechanical, avionics, etc)
            child = QTreeWidgetItem(parent)
            child.setFlags(child.flags() | Qt.ItemIsTristate | Qt.ItemIsUserCheckable)
            child.setText(0, section_header[j])
            # child.setExpanded(True)
            while k < len(list):
                if list[k][0] in section_header:
                    # when the while loop encounters the first title, continue the loop
                    if (list[k][0] == section_header[0]):
                        k += 1
                        continue
                    # when the while loop encounters the next titles, breaks so that the value of j increases
                    k += 1 
                    break
                # when the list contains empty cells, skip the cell and continue
                elif list[k][0] == '':
                    k += 1
                    continue
                # add the list items to the treewidgetitem
                # grandchild refers to the items under each section (wing nuts, tail nuts, etc)
                grandchild = QTreeWidgetItem(child)
                grandchild.setText(0, list[k][0])
                grandchild.setText(1, list[k][1])
                grandchild.setCheckState(0, Qt.Unchecked) # uncheck everything
                k += 1

    def excel_parser(self, file_name):
        with open(file_name, 'r') as file:
            checklist = []
            reader = csv.reader(file)
            for row in reader:
                checklist.append(row)
            return checklist            

    def ok_clicked(self):
        if self.BPO_header.checkState(0) != 2 or self.BTO_header.checkState(0) != 2:
            self.dialog_window("Some items in the checklist are still unchecked", "Do you still want to continue?", True)
        else:
            self.close()
        

    def cancel_clicked(self):
        if self.BPO_header.checkState(0) != 0 or self.BTO_header.checkState(0) != 0:
            self.dialog_window('Some of your items are checked. Cancelling will uncheck all your items', 'Do you still want to continue?', False)
        else:
            self.BTO_header.setCheckState(0, Qt.Unchecked)
            self.BPO_header.setCheckState(0, Qt.Unchecked)
            self.close()
    
    def load_clicked(self):
        filenames = QFileDialog.getOpenFileNames(
            self, self.tr('Load from Files'), '.', self.tr('csv files {.csv} (*.csv)'))
        for filename in filenames[0]:
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
    
    def remove_widget(self):
        self.layout.removeWidget(self.tree_widget)
        self.tree_widget.deleteLater()
        self.buttons_layout.removeWidget(self.ok_button)
        self.buttons_layout.removeWidget(self.cancel_button)
        self.buttons_layout.removeWidget(self.load_button)
        self.ok_button.deleteLater()
        self.cancel_button.deleteLater()
        self.load_button.deleteLater()
    
    def dialog_window(self, message, detail, check):
        print('show dialog is present')
        self.message = QMessageBox()
        self.message.setIcon(QMessageBox.Warning)
        self.message.setText(message)
        self.message.setInformativeText(detail)
        self.message.setWindowTitle("Items are unchecked")
        self.message.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        self.message.show()
        if check == True:
            self.message.buttonClicked.connect(self.message_action)
        else:
            self.message.buttonClicked.connect(self.message_action_uncheck)
    
    def message_action(self, i):
        if i.text() == '&Yes':
            self.close()
        else:
            self.message.close()
    
    def message_action_uncheck(self, i):
        self.response = i.text()
        if self.response == '&Yes':
            self.BTO_header.setCheckState(0, Qt.Unchecked)
            self.BPO_header.setCheckState(0, Qt.Unchecked)
            self.close()
        else:
            self.message.close()


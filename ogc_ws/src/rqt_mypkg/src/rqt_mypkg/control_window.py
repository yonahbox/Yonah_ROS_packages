#!/usr/bin/env python3
class ControlWindow(QWidget):
    def __init__(self):
        super(ChecklistWindow, self).__init__()
        self.setWindowTitle("Main Ground Control Station")
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
        

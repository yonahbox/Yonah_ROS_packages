#TODO create a separate confirmation_window class. This file is currently unused by the plugin

# from python_qt_binding import loadUi
# from python_qt_binding.QtWidgets import QMessageBox, QWidget
# from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, Slot

# class ConfirmationWindow(QWidget):
#     def __init__(self, messages, additional_info = ''):
#         super(ChecklistWindow, self).__init__()
#         print('confirmation received')
        
#         self.message = QMessageBox()
#         self.message.setWindowTitle("MessageBox demo")
#         self.message.setIcon(QMessageBox.Warning)
#         self.message.setText(messages)
#         self.message.setInformativeText(additional_info)
#         self.message.setStandardButtons(QMessageBox.No | QMessageBox.Yes)
        
#         self.show()
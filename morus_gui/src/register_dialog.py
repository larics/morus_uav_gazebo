from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class RegisterUser(QDialog):
    """
    Class implementina QDialog functionality for registering users.
    Class variables nickname, first_name and last_name are used for storing user input.
    """

    def __init__(self):
        super(self.__class__, self).__init__()

        self.setupUI()
        self.setupCallbacks()

        self.nickname = ""
        self.first_name = ""
        self.last_name = ""

    def setupCallbacks(self):
        """
        Setup QDialog button callbacks
        """

        self.cancel_button.clicked.connect(self.close)
        self.ok_button.clicked.connect(self.submit_close)

    def submit_close(self):
        """
        Save entered user information in the respective class variables
        and close QDialog with accept()
        """
        # TODO: Handle case where users does not enter information fully
        self.nickname = str (self.h_layout_nick.itemAt(1).widget().text() )
        self.first_name = str (self.h_layout_fname.itemAt(1).widget().text() )
        self.last_name = str (self.h_layout_lname.itemAt(1).widget().text() )

        if not self.nickname or not self.first_name or not self.last_name:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Resigtration error")
            msg.setInformativeText('Please fully complete user registration')
            msg.setWindowTitle("Error")
            msg.exec_()
        else:
            self.accept()

    def setupUI(self):
        """
        Setup UI layout
        """

        self.h_layout_nick = self.get_label_edit_layout("Enter nickname: ")
        self.h_layout_fname = self.get_label_edit_layout("Enter first name: ")
        self.h_layout_lname = self.get_label_edit_layout("Enter last name: ")

        # Create a button layout
        self.ok_button = QPushButton("OK")
        self.cancel_button = QPushButton("Cancel")
        
        h_layout_btns = QHBoxLayout()
        h_layout_btns.addWidget(self.ok_button)
        h_layout_btns.addWidget(self.cancel_button)

        # Add all layouts to vertical layout
        v_layout = QVBoxLayout()
        v_layout.setSpacing(5)
        v_layout.addLayout(self.h_layout_nick)
        v_layout.addLayout(self.h_layout_fname)
        v_layout.addLayout(self.h_layout_lname)
        v_layout.addLayout(h_layout_btns)

        self.setWindowTitle("User registration")
        self.setLayout(v_layout)

    def get_label_edit_layout(self, name):
        """
        Generator function for horizontal layout container with
        Label and Edit Box.
        """

        edit_box = QLineEdit()
        edit_box.setFixedWidth(100)

        h_layout = QHBoxLayout()
        h_layout.setSpacing(50)
        h_layout.addWidget(QLabel(name))
        h_layout.addWidget(edit_box)

        return h_layout
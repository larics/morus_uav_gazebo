from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys

from run_gazebo import RunGazeboSimulator

class MorusGUI(QtWidgets.QWidget):
    
    INFO_TEXT_INIT = "Please register before playing"
    DEFAULT_BTN_WIDTH = 200

    def __init__(self):
        super(self.__class__, self).__init__()
        
        self.setupUI()

        self.start_button.clicked.connect(self.btn1_cb)
        self.stop_button.clicked.connect(self.btn2_cb)

    def setupUI(self):
        self.register_button = QPushButton("New User")
        self.start_button = QPushButton("Start")
        self.start_button.setEnabled(False)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        
        logo_label = QLabel()
        pixmap = QPixmap('../resources/MORUS.png')
        logo_label.setPixmap(pixmap)
        logo_label.setAlignment(Qt.AlignCenter)

        self.info_label = QLabel()
        self.info_label.setAlignment(Qt.AlignCenter)
        newfont = QtGui.QFont("Aerial", 18, QtGui.QFont.Bold) 
        self.info_label.setFont(newfont)
        self.info_label.setText(MorusGUI.INFO_TEXT_INIT)
        
        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(logo_label)
        layout.addWidget(self.info_label)
        layout.addWidget(self.register_button)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Drone Days 2019")
        self.show()

    def btn1_cb(self):
        self.workThread = RunGazeboSimulator()
        self.workThread.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def btn2_cb(self):
        print("Button2 clicked")
        self.workThread.stop_simlation()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys

from run_gazebo import RunGazeboSimulator

class MorusGUI(QtWidgets.QWidget):
    
    def __init__(self):
        super(self.__class__, self).__init__()
        
        self.setupUI()

        self.button_1.clicked.connect(self.btn1_cb)
        self.button_2.clicked.connect(self.btn2_cb)

    def setupUI(self):
        self.button_1 = QPushButton('Top')
        self.button_2 = QPushButton('Bottom')
        self.button_2.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.button_1)
        layout.addWidget(self.button_2)

        self.setLayout(layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle('Morus GUI')
        self.show()

    def btn1_cb(self):

        self.workThread = RunGazeboSimulator()
        self.workThread.start()
        self.button_1.setEnabled(False)
        self.button_2.setEnabled(True)

    def btn2_cb(self):
        print("Button2 clicked")
        self.workThread.stop_simlation()
        self.button_1.setEnabled(True)
        self.button_2.setEnabled(False)

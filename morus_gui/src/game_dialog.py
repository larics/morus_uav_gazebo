from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from std_msgs.msg import Int8, Float64

from loop_thread import LoopMonitorThread

class LoopDialog(QDialog):
    """
    This class implements QDialog functionality. It shows the user 
    currently available game information and score
    """

    NOT_RUNNING_MSG = "Move the UAV to start the game"
    RUNNING_MSG = "Game started. Good luck!"
    FINISHED_MSG = "Game Over!"
    
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUI()
        
    def setupUI(self):
        self.label = QLabel("Press play when ready")
        self.edit1 = QLineEdit()
        self.edit2 = QLineEdit()
        self.p_button = QPushButton("Play")
        self.p_button.clicked.connect(self.run_loop)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.label)
        v_layout.addWidget(self.edit1)
        v_layout.addWidget(self.edit2)
        v_layout.addWidget(self.p_button)

        self.setLayout(v_layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Game Loop")
        self.show()

    def run_loop(self):
        self.p_button.setEnabled(False)
        self.p_button.setVisible(False)
        self.label.setText("Game starting...")
        
        self.monitor_thread = LoopMonitorThread(self)
        self.monitor_thread.start()     

    @pyqtSlot(float)
    def updateDistance(self, arg1):
        print("LoopDialog: Update distance")
        pass

    @pyqtSlot(float)
    def updateTime(self, arg1):
        print("LoopDialog: Update time")
        pass

    @pyqtSlot(int)
    def updateGameStatus(self, arg1):
        print("LoopDialog: Update Game")
        if arg1 == LoopMonitorThread.NOT_RUNNING:
            self.label.setText(LoopDialog.NOT_RUNNING_MSG)

        elif arg1 == LoopMonitorThread.RUNNING:
            self.label.setText(LoopDialog.RUNNING_MSG)

        elif arg1 == LoopMonitorThread.FINISHED:
            self.label.setText(LoopDialog.FINISHED_MSG)

    
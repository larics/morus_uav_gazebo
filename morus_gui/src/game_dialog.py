from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from std_msgs.msg import Int8, Float64

from loop_thread import LoopMonitor

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
        self.info_label = QLabel("Press play when ready")
        newfont = QtGui.QFont("Aerial", 15, QtGui.QFont.Bold) 
        self.info_label.setFont(newfont)

        self.time_label = QLabel()
        self.dist_label = QLabel()
        self.p_button = QPushButton("Play")
        self.p_button.clicked.connect(self.run_loop)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.info_label)
        v_layout.addWidget(self.time_label)
        v_layout.addWidget(self.dist_label)
        v_layout.addWidget(self.p_button)

        self.setLayout(v_layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Game Loop")
        self.show()

    def run_loop(self):
        self.p_button.setEnabled(False)
        self.p_button.setVisible(False)
        self.info_label.setText("Game starting...")
        
        self.monitor = LoopMonitor()
        self.monitor.status_signal.connect(self.updateGameStatus)
        self.monitor.dist_signal.connect(self.updateDistance)
        self.monitor.time_signal.connect(self.updateTime)
        self.monitor.start_node()     

    @pyqtSlot(float)
    def updateDistance(self, arg1):
        print("LoopDialog: Update distance")
        self.dist_label.setText(str(arg1))

        self.refresh_ui()

    @pyqtSlot(float)
    def updateTime(self, arg1):
        print("LoopDialog: Update time")
        self.time_label.setText(str(arg1))
        
        self.refresh_ui()

    @pyqtSlot(int)
    def updateGameStatus(self, arg1):
        print("LoopDialog: Update Game")
        if arg1 == LoopMonitor.NOT_RUNNING:
            self.info_label.setText(LoopDialog.NOT_RUNNING_MSG)

        elif arg1 == LoopMonitor.RUNNING:
            self.info_label.setText(LoopDialog.RUNNING_MSG)

        elif arg1 == LoopMonitor.FINISHED:
            self.info_label.setText(LoopDialog.FINISHED_MSG)

        self.refresh_ui()

    def refresh_ui(self):
        self.update()
        self.repaint()
    
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from std_msgs.msg import Int8, Float64

class LoopDialog(QDialog):

    NOT_RUNNING = 1
    RUNNING = 2
    FINISHED = 3

    NOT_RUNNING_MSG = "Move the UAV to start the game"
    RUNNING_MSG = "Game started. Good luck!"
    FINISHED_MSG = "Game Over!"

    def __init__(self):
        super(self.__class__, self).__init__()

        self.setupUI()
        
        self.running_status = -1

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

        self.label.setText("Game starting in 5 seconds...")
        """
        rospy.init_node("game_monitor")
        rospy.Subscriber("/game_loop/running", Int8, self.status_callback)
        rospy.Subscriber("/game_loop/distance", Float64, self.distance_callback)
        rospy.Subscriber("/game_loop/elapsed_time", Float64, self.time_callback)

        while not rospy.is_shutdown() and not self.running_status == LoopDialog.FINISHED:
            print("Spinning")
            self.update()
            rospy.sleep(0.5)

        print("Game finished")
        self.accept()
        """

    def setupCallbacks(self):
        pass

    def status_callback(self, msg):
        self.running_status = msg.data

        if self.running_status == LoopDialog.NOT_RUNNING:
            self.label.setText(LoopDialog.NOT_RUNNING_MSG)

        elif self.running_status == LoopDialog.RUNNING:
            self.label.setText(LoopDialog.RUNNING_MSG)

        elif self.running_status == LoopDialog.FINISHED:
            self.label.setText(LoopDialog.FINISHED_MSG)

    def distance_callback(self, msg):
        self.edit1.setText(str(msg.data))

    def time_callback(self, msg):
        self.edit2.setText(str(msg.data))
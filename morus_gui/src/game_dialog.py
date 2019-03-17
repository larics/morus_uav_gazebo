from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from std_msgs.msg import Int8, Float64

from loop_monitor import LoopMonitor

class LoopDialog(QDialog):
    """
    This class implements QDialog functionality. It shows the user 
    currently available game information and score. Also providing
    options to start or quit the game as well as save the achieved
    score.
    """

    NOT_RUNNING_MSG = "Move the UAV to start the game"
    RUNNING_MSG = "Game started. Good luck!"
    FINISHED_MSG = "Game Over!"
    
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUI()
        self.setupCallbacks()

        self.monitor = LoopMonitor()
        self.monitor.status_signal.connect(self.updateGameStatus)
        self.monitor.dist_signal.connect(self.updateDistance)
        self.monitor.time_signal.connect(self.updateTime)
        
        
    def setupCallbacks(self):
        """
        Setup all callbacks for UI elements.
        """

        self.start_button.clicked.connect(self.run_loop)
        self.record_button.clicked.connect(self.record_btn_callback)
        self.quit_button.clicked.connect(self.quit_btn_callback)

    def setupUI(self):
        """
        Setup visual UI elements with no funcitonality.
        """

        self.info_label = QLabel("Press play when ready")
        newfont = QtGui.QFont("Aerial", 15, QtGui.QFont.Bold) 
        self.info_label.setFont(newfont)
        self.info_label.setAlignment(Qt.AlignCenter)

        labelfont = QtGui.QFont("Aerial", 13, QtGui.QFont.Bold)
        self.remaining_time_label = QLabel()
        self.dist_label = QLabel()

        self.remaining_time_label.setFont(labelfont)
        self.dist_label.setFont(labelfont)

        self.remaining_time_label.setFixedWidth(100)
        self.dist_label.setFixedWidth(100)

        t_label = QLabel("Time remaining: ")
        t_label.setFont(labelfont)
        d_label = QLabel("Achieved distance: ")
        d_label.setFont(labelfont)
        
        t_layout = QHBoxLayout()
        t_layout.addWidget(t_label)
        t_layout.addWidget(self.remaining_time_label)
        t_layout.setSpacing(50)

        d_layout = QHBoxLayout()
        d_layout.addWidget(d_label)
        d_layout.addWidget(self.dist_label)
        d_layout.setSpacing(50)

        btn_layout = QHBoxLayout()
        self.record_button = QPushButton("Save Score")
        self.quit_button = QPushButton("Quit")
        self.record_button.setEnabled(False)
        self.record_button.setVisible(False)
        btn_layout.addWidget(self.record_button)
        btn_layout.addWidget(self.quit_button)

        self.start_button = QPushButton("Play")

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.info_label)
        v_layout.addLayout(t_layout)
        v_layout.addLayout(d_layout)
        v_layout.addWidget(self.start_button)
        v_layout.addLayout(btn_layout)

        self.setLayout(v_layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Game Loop")
        
        # Disable the exit button
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~QtCore.Qt.WindowCloseButtonHint)
        
        self.show()

    def run_loop(self):
        """
        Run the game loop by starting the loop_monitor node. 
        """

        self.start_button.setEnabled(False)
        self.start_button.setVisible(False)

        self.quit_button.setEnabled(False)
        self.quit_button.setVisible(False)

        self.info_label.setText("Game starting...")
        self.monitor.start_node()     

    def record_btn_callback(self):
        """
        Record player achieved score when the game finishes
        """

        self.monitor.stop_node()
        self.accept()

    def quit_btn_callback(self):
        """
        Close the game.
        """

        self.monitor.stop_node()
        self.close()

    def keyPressEvent(self, event):
        """
        Disable exiting the QDialog when pressing escape key
        """

        key = event.key()
        if key == Qt.Key_Escape:
            # Don't close on escape
            pass

    @pyqtSlot(float)
    def updateDistance(self, arg1):
        # print("LoopDialog: Update distance")
        self.dist_label.setText("{0:.2f} m".format(arg1))
        self.refresh_ui()

    @pyqtSlot(float)
    def updateTime(self, arg1):
        # print("LoopDialog: Update time")
        self.remaining_time_label.setText("{0:.2f} s".format(arg1))
        self.refresh_ui()

    @pyqtSlot(int)
    def updateGameStatus(self, arg1):
        # print("LoopDialog: Update Game")
        if arg1 == LoopMonitor.NOT_RUNNING:
            self.info_label.setText(LoopDialog.NOT_RUNNING_MSG)

        elif arg1 == LoopMonitor.RUNNING:
            self.info_label.setText(LoopDialog.RUNNING_MSG)

        elif arg1 == LoopMonitor.FINISHED:
            self.info_label.setText(LoopDialog.FINISHED_MSG)
            self.record_button.setEnabled(True)
            self.record_button.setVisible(True)

            self.quit_button.setEnabled(True)
            self.quit_button.setVisible(True)

        self.refresh_ui()

    def refresh_ui(self):
        """
        Refresh the UI elements by repainting and adjusting size.
        """

        self.repaint()
        self.adjustSize()
    
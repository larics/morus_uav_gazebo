from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys
import rospkg 

from run_gazebo import RunGazeboSimulator
from register_dialog import RegisterUser
from score_tracker import ScoreTracker

class UIState():
    NO_USER = 0
    USER_REGISTERED = 1

class MorusGUI(QWidget):
    
    INFO_TEXT_INIT = "Please register before playing"
    DEFAULT_BTN_WIDTH = 200
    LAUNCH_FILE = "/launch/morus_multirotor_height_attitude_ctl.launch"
    PICTURE_FILE = "/resources/MORUS.png"

    def __init__(self):
        super(self.__class__, self).__init__()
        
        self.setupUI()
        self.setupCallbacks()
        self.current_state = UIState.NO_USER
        self.score_tracker = ScoreTracker()

    def setupCallbacks(self):
        """
        Setup all needed callbacks for GUI objects.
        """

        self.start_button.clicked.connect(self.start_btn_callback)
        self.stop_button.clicked.connect(self.stop_btn_callback)
        self.register_button.clicked.connect(self.register_btn_callback)

    def setupUI(self):
        """
        Setup all visual GUI-related objects with no functionality
        """

        # Create all buttons
        self.register_button = QPushButton("New User")
        self.start_button = QPushButton("Start")
        self.start_button.setEnabled(False)
        self.stop_button = QPushButton("Stop")
        self.stop_button.setEnabled(False)
        
        # Create a visual label
        rospack = rospkg.RosPack()
        path = rospack.get_path("morus_gui")
        pic_path = path + MorusGUI.PICTURE_FILE
         
        logo_label = QLabel()
        pixmap = QPixmap(pic_path)
        logo_label.setPixmap(pixmap)
        logo_label.setAlignment(Qt.AlignCenter)

        # Create information label
        self.info_label = QLabel()
        self.info_label.setAlignment(Qt.AlignCenter)
        newfont = QtGui.QFont("Aerial", 15, QtGui.QFont.Bold) 
        self.info_label.setFont(newfont)
        self.info_label.setText(MorusGUI.INFO_TEXT_INIT)

        # Setup vertical layout
        layout = QVBoxLayout()
        layout.addWidget(logo_label)
        layout.addWidget(self.info_label)
        layout.addWidget(self.register_button)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Drone Days 2019")
        self.show()

    def register_btn_callback(self):
        """
        Generate and run a user registration form.
        """

        dialog_box = RegisterUser()
        if dialog_box.exec_():
            
            if not self.score_tracker.add_initial_entry(
                dialog_box.nickname,
                dialog_box.first_name,
                dialog_box.last_name):

                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Registration error")
                msg.setInformativeText(
                    "User with nickname {} already exists, please chose a different nickname.".format(
                        dialog_box.nickname))
                msg.setWindowTitle("Error")
                msg.exec_()

    def start_btn_callback(self):
        """
        Start button callback.rospack = rospkg.RosPack()
        path = rospack.get_path("morus_gazebo")
        launch_path = path + MorusGUI.LAUNCH_FILE

        Start the Gazebo simulation.
        """

        rospack = rospkg.RosPack()
        path = rospack.get_path("morus_gazebo")
        launch_path = path + MorusGUI.LAUNCH_FILE

        self.workThread = RunGazeboSimulator(launch_path)
        self.workThread.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_btn_callback(self):
        """
        Stop button callback.
        Stop the Gazebo simulation.
        """

        self.workThread.stop_simlation()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

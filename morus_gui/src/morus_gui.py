from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys
import rospkg

from run_gazebo import RunGazeboSimulator
from register_dialog import RegisterUser
from score_tracker import ScoreTracker
from game_dialog import LoopDialog

class UIState():
    NO_USER = 0
    USER_REGISTERED = 1

class MorusGUI(QWidget):
    
    INFO_TEXT_INIT = "Please register before playing"
    INFO_TEXT_USER = "Welcome {}!"
    DEFAULT_BTN_WIDTH = 200
    PACKAGE = "morus_gui"
    LAUNCH_FILE = "/launch/drone_days.launch"
    PICTURE_FILE = "/resources/MORUS.png"

    def __init__(self):
        super(self.__class__, self).__init__()
        
        self.setupUI()
        self.setupCallbacks()
        self.current_state = UIState.NO_USER
        self.score_tracker = ScoreTracker()
        self.current_user = None

    def setupCallbacks(self):
        """
        Setup all needed callbacks for GUI objects.
        """

        self.start_button.clicked.connect(self.start_btn_callback)
        self.stop_button.clicked.connect(self.stop_btn_callback)
        self.register_button.clicked.connect(self.register_btn_callback)
        self.exusr_button.clicked.connect(self.exusr_btn_callback)
        self.exit_button.clicked.connect(self.exit_btn_callback)

    def setupUI(self):
        """
        Setup all visual GUI-related objects with no functionality
        """

        # Create all buttons
        self.register_button = QPushButton("New User")
        self.exusr_button = QPushButton("Existing User")
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop")
        self.exit_button = QPushButton("Logout")       

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

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.register_button)
        btn_layout.addWidget(self.exusr_button)

        # Setup vertical layout
        layout = QVBoxLayout()
        layout.addWidget(logo_label)
        layout.addWidget(self.info_label)
        layout.addLayout(btn_layout)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.exit_button)

        self.setLayout(layout)
        self.setGeometry(300, 300, 250, 150)
        self.setWindowTitle("Drone Days 2019")
        self.show()

        self.set_UI_state(UIState.NO_USER)

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
            
            else:
                self.set_UI_state(UIState.USER_REGISTERED, dialog_box.nickname)

    def set_UI_state(self, new_state, nick=None):

        if (new_state == UIState.USER_REGISTERED):
            self.global_enable_btns(True)
            self.current_user = nick
            self.info_label.setText(MorusGUI.INFO_TEXT_USER.format(nick))

        if (new_state == UIState.NO_USER):
            self.current_user = None
            self.info_label.setText(MorusGUI.INFO_TEXT_INIT)
            self.global_enable_btns(False)

    def global_enable_btns(self, enable):
        """
        Set global button properties.
        """

        self.register_button.setEnabled(not enable)
        self.register_button.setVisible(not enable)

        self.exusr_button.setEnabled(not enable)
        self.exusr_button.setVisible(not enable)

        self.exit_button.setEnabled(enable)
        self.exit_button.setVisible(enable)

        self.start_button.setEnabled(enable)
        self.start_button.setVisible(enable)

        self.stop_button.setEnabled(enable)
        self.stop_button.setVisible(enable)

    def start_btn_callback(self):
        """
        Start button callback.rospack = rospkg.RosPack()
        path = rospack.get_path("morus_gazebo")
        launch_path = path + MorusGUI.LAUNCH_FILE

        Start the Gazebo simulation.
        """

        rospack = rospkg.RosPack()
        path = rospack.get_path(MorusGUI.PACKAGE)
        launch_path = path + MorusGUI.LAUNCH_FILE

        self.workThread = RunGazeboSimulator(launch_path)
        self.workThread.start()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)


        # Start the LoopDialog - Disable main window functionality
        loop_dialog = LoopDialog()
        self.setEnabled(False)

        if loop_dialog.exec_():
            # Record score from here
            pass

        self.stop_btn_callback()
        self.setEnabled(True)

    def stop_btn_callback(self):
        """
        Stop button callback.
        Stop the Gazebo simulation.
        """

        self.workThread.stop_simlation()

        if self.workThread.isRunning():
            self.workThread.terminate()

        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def exusr_btn_callback(self):
        """
        Generate existing user form.
        """

        usr_list = self.score_tracker.get_user_list()
        user, ok = QInputDialog.getItem(self, "Select Existing User", 
         "List of Existing Users", usr_list, 0, False)

        if ok and user:
            self.set_UI_state(UIState.USER_REGISTERED, user)

    def exit_btn_callback(self):
        """
        Logout current user.
        """
        self.set_UI_state(UIState.NO_USER)
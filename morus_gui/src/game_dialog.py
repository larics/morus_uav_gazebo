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
    FINISHED_MSG = "Game Over! Score: {}"
    
    def __init__(self, loop_monitor):
        super(self.__class__, self).__init__()
        self.setupUI()
        self.setupCallbacks()
        self.setAttribute(Qt.WA_DeleteOnClose)

        self.loop_monitor = loop_monitor
        self.loop_monitor.status_signal.connect(self.updateGameStatus)
        self.loop_monitor.dist_signal.connect(self.updateDistance)
        self.loop_monitor.time_signal.connect(self.updateTime)
        self.loop_monitor.image_signal.connect(self.updatePicture)

        self.loop_thread = False
        
    def __del__(self):
        print("LoopDialog: Destructor called")

        if self.loop_thread and self.loop_monitor:
            print("LoopDialog: Stopping loop monitors")
            self.loop_thread.quit()
            self.loop_thread.wait()

        if self.loop_monitor:
            self.loop_monitor.status_signal.disconnect()
            self.loop_monitor.dist_signal.disconnect()
            self.loop_monitor.time_signal.disconnect()
            self.loop_monitor.image_signal.disconnect()


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

        self.cam_label = QLabel()
        self.camera_qpix = QPixmap()
        self.cam_label.setAlignment(Qt.AlignCenter)
        self.cam_label.setVisible(False)

        v_layout = QVBoxLayout()
        v_layout.addWidget(self.info_label)
        v_layout.addLayout(t_layout)
        v_layout.addLayout(d_layout)
        v_layout.addWidget(self.cam_label)
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


        self.cam_label.setVisible(True)
        self.info_label.setText("Game starting...")
        
        self.loop_thread = QThread()
        self.loop_monitor.moveToThread(self.loop_thread)
        self.loop_thread.started.connect(self.loop_monitor.start_node)
        self.loop_thread.start()

    def record_btn_callback(self):
        """
        Record player achieved score when the game finishes
        """

        self.loop_monitor.stop_node()
        self.final_score = self.loop_monitor.final_score
        self.final_time = self.loop_monitor.final_time
        self.accept()

    def quit_btn_callback(self):
        """
        Close the game.
        """

        # The QWidget widget is the base class of all user interface objects in PyQt4.
        w = QWidget()
        result = QMessageBox.question(
            w, 'Message', 
            "Are you sure you want to quit? Score will not be saved.", 
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if result == QMessageBox.No:
            return 

        self.loop_monitor.stop_node()
        self.final_score = 0
        self.final_time = 0
        self.close()

    def keyPressEvent(self, event):
        """
        Disable exiting the QDialog when pressing escape key
        """

        key = event.key()
        if key == Qt.Key_Escape:
            # Don't close on escape
            pass

    @pyqtSlot(bytes)
    def updatePicture(self, arg1):
        check = self.camera_qpix.loadFromData(bytes(arg1))
        self.cam_label.setPixmap(self.camera_qpix)

        self.refresh_ui()

    @pyqtSlot(float)
    def updateDistance(self, arg1):
        if self.loop_monitor.running_status <= LoopMonitor.NOT_RUNNING:
            self.dist_label.setText("Wait...")
        else:
            self.dist_label.setText("{0:.2f} m".format(arg1))

        self.refresh_ui()

    @pyqtSlot(float)
    def updateTime(self, arg1):
        if self.loop_monitor.running_status <= LoopMonitor.NOT_RUNNING:
            self.remaining_time_label.setText("Wait...")
        else:
            self.remaining_time_label.setText("{0:.2f} s".format(arg1))

        self.refresh_ui()

    @pyqtSlot(int)
    def updateGameStatus(self, arg1):
        if arg1 == LoopMonitor.NOT_RUNNING:
            self.info_label.setText(LoopDialog.NOT_RUNNING_MSG)

        elif arg1 == LoopMonitor.RUNNING:
            self.info_label.setText(LoopDialog.RUNNING_MSG)

        elif arg1 == LoopMonitor.FINISHED:
            self.loop_monitor.calculate_final_score()
            self.info_label.setText(
                LoopDialog.FINISHED_MSG.format(self.loop_monitor.final_score))
            self.record_button.setEnabled(True)
            self.record_button.setVisible(True)

            self.quit_button.setEnabled(True)
            self.quit_button.setVisible(True)

        self.refresh_ui()

    def refresh_ui(self):
        """
        Refresh the UI elements by repainting and adjusting size.
        """

        self.update()
        self.repaint()
        self.adjustSize()
    
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import rospy
from std_msgs.msg import Int8, Float64, Bool
from std_srvs.srv import Empty

import sys, time

class LoopMonitor(QObject):
    """
    This class implements a QThread class. Its main task is to run
    a ROS node used to monitor game activities and emit signals back towards
    the 
    """
    
    NOT_RUNNING = 1
    RUNNING = 2
    FINISHED = 3

    status_signal = pyqtSignal(int)
    dist_signal = pyqtSignal(float)
    time_signal = pyqtSignal(float)

    def __init__(self):
        super(self.__class__, self).__init__()
        """
        Initialize all the signals for this thread. Communication to outside
        will be made only through signals.
        """
        print("Inside loop monitor thread")
        self.external_enable = True

    def initialize_node(self):
        """
        Initialize node subscribers.
        """

        self.running_status = -1
        self.dist = -1
        self.time = -1

        rospy.init_node("game_monitor")
        rospy.Subscriber("/game_loop/running", Int8, self.status_callback)
        rospy.Subscriber("/game_loop/distance", Float64, self.distance_callback)
        rospy.Subscriber("/game_loop/remaining_time", Float64, self.time_callback)
      
    def start_node(self):
        """
        Start the main node loop. Emit Monitor information back to the main thread.
        """
        
        print("LoopMonitor: Initializing node")
        self.initialize_node()
        
        print("LoopMonitor: Starting node")
        while \
            not rospy.is_shutdown() and \
            not self.running_status == LoopMonitor.FINISHED and \
            self.external_enable:

            rospy.sleep(0.01)
            #print(self.running_status, self.dist, self.time)

            self.status_signal.emit(
                int(self.running_status))
            self.dist_signal.emit(
                float(self.dist))
            self.time_signal.emit(
                float(self.time))

        print("LoopMonitor: Pausing Simulation")
        service_call = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        service_call()
        print("LoopMonitor: Node finished")

    def stop_node(self):
        self.external_enable = False

    def status_callback(self, msg):
        self.running_status = msg.data

    def distance_callback(self, msg):
        self.dist = msg.data

    def time_callback(self, msg):
        self.time = msg.data
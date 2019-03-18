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
        print("LoopMonitor: Inside loop monitor constructor")
        self.external_enable = True

        rospy.init_node("game_monitor")
        self.start_teleop_pub = rospy.Publisher("/game_loop/teleop_status", Bool, queue_size=1)
        rospy.Subscriber("/game_loop/running", Int8, self.status_callback)
        rospy.Subscriber("/game_loop/distance", Float64, self.distance_callback)
        rospy.Subscriber("/game_loop/remaining_time", Float64, self.time_callback)

    def initialize_node(self):
        """
        Initialize node subscribers.
        """

        self.running_status = -1
        self.achieved_distance = -1
        self.remaining_time = -1
        self.external_enable = True

    def start_node(self):
        """
        Start the main node loop. Emit Monitor information back to the main thread.
        """
        
        print("LoopMonitor: Initializing node")
        self.initialize_node()
        
        print("LoopMonitor: Is rospy shutdown? {}".format(rospy.is_shutdown()))
        print("LoopMonitor: Is loop monitor finished? {}".format(self.running_status == LoopMonitor.FINISHED))
        print("LoopMonitor: Is enabled? {}".format(self.external_enable))

        print("LoopMonitor: Starting node")
        while \
            not rospy.is_shutdown() and \
            not self.running_status == LoopMonitor.FINISHED and \
            self.external_enable:

            rospy.sleep(0.01)
            self.publish_teleop_status(True)
            print("LoopMonitor: Hello from loop")
            self.status_signal.emit(int(self.running_status))
            self.dist_signal.emit(float(self.achieved_distance))
            self.time_signal.emit(float(self.remaining_time))

            print("\n\n")

        try:
            print("LoopMonitor: Trying to pause Simulation")
            service_call = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
            service_call()
        except: 
            print("LoopMonitor: Unable to pause Gazebo simulation")

        self.publish_teleop_status(False)
        print("LoopMonitor: Node finished")
        #rospy.signal_shutdown("LoopMonitor shutting down")

   
    def stop_node(self):
        """
        Perform all actions requiref dor stoping the node
        """

        print("LoopMonitor: Stopping loop_monitor node")
        self.external_enable = False
        self.status_signal.emit(
            int(LoopMonitor.FINISHED))

    def publish_teleop_status(self, status):
        msg = Bool()
        msg.data = status
        self.start_teleop_pub.publish(msg)

    def status_callback(self, msg):
        self.running_status = msg.data

    def distance_callback(self, msg):
        print("LoopMonitor: Hello from callback")
        self.achieved_distance = msg.data

    def time_callback(self, msg):
        self.remaining_time = msg.data

        # Stop the game if time runs out
        if self.remaining_time <= 0:
            self.stop_node()
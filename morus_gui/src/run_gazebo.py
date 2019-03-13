from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import * 
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import sys, time
import multiprocessing as mp
from multiprocessing import Process

import roslaunch
import rospy

class RunGazeboSimulator(QThread):
    def __init__(self):
        QThread.__init__(self)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch_sim = roslaunch.parent.ROSLaunchParent(uuid, 
        ["/home/lmark/Documents/catkin_ws/src/morus_uav_gazebo/morus_gazebo/launch/morus_multirotor_height_attitude_ctl.launch"])
        
    def __del__(self):
        self.wait()

    def start_simulation(self):
        self.launch_sim.start()
        
        try:
            self.launch_sim.spin()
        except:
            pass

        self.launch_sim.shutdown() 

    def stop_simlation(self):
        self.sim_process.terminate()
        self.sim_process.join()

    def run(self):
        self.sim_process = Process(target=self.start_simulation)
        self.sim_process.start()
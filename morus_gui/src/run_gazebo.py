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
    """
    Class for initializing Gazebo simulator.
    Uses roslaunch API to execute a roslaunch file inside a Process.
    """

    def __init__(self, path):
        QThread.__init__(self)

        # Initialize roslaunch object
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch_sim = roslaunch.parent.ROSLaunchParent(uuid, 
        [path])
        
    def __del__(self):
        self.wait()

    def start_simulation(self):
        """
        Start simulation using the roslaunch API.
        """

        print("Inside process: Starting Simulation")
        self.launch_sim.start()
        
        try:
            self.launch_sim.spin()
        except:
            print("Inside prcoess: Exception occured")
            pass

        print("Inside process: Shutting down roslaunch")
        self.launch_sim.shutdown()

    def stop_simlation(self):
        """
        Stop and joint the Process.
        """

        self.sim_process.terminate()
        self.sim_process.join()

    def run(self):
        """
        Create a new Process object, append start_simulation handle
        and start it.
        """

        self.sim_process = Process(target=self.start_simulation)
        self.sim_process.start()
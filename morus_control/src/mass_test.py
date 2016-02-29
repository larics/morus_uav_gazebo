#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64
from mav_msgs.msg import MotorSpeed
from mav_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
from mav_msgs.cfg import MavAttitudeCtlParamsConfig
import math

if __name__ == '__main__':

    rospy.init_node('morus_test_mass')
    ros_rate = rospy.Rate(10)                 # attitude control at 100 Hz

    pub_mass0 = rospy.Publisher('/morus/movable_mass_0_position_controller/command', Float64, queue_size=1)
    pub_mass1 = rospy.Publisher('/morus/movable_mass_1_position_controller/command', Float64, queue_size=1)
    pub_mass2 = rospy.Publisher('/morus/movable_mass_2_position_controller/command', Float64, queue_size=1)
    pub_mass3 = rospy.Publisher('/morus/movable_mass_3_position_controller/command', Float64, queue_size=1)

    amplitude = 0.1
    pos_ref_msg = Float64()
    pos_ref_msg.data = amplitude
    sign = -1

    while True:
        ros_rate.sleep()
        sign *= -1
        pos_ref_msg.data = amplitude * sign
        pub_mass3.publish(pos_ref_msg)
        #pos_ref_msg.data *= -1
        pub_mass1.publish(pos_ref_msg)


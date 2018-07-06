#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pid_smc import PID
from trajectory_msgs.msg import MultiDOFJointTrajectory
from first_order_filter import FirstOrderFilter
from morus_msgs.msg import SMCStatus
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from morus_control.cfg import SmcMmcuavPositionCtlParamsConfig
from nonlinear_blocks import deadzone, saturation
from sensor_msgs.msg import Imu


class SmcAttitudeController:

    def __init__(self):

        self.sleep_sec = 1
        self.first_measurement = False

        # Controller rate
        self.controller_rate = 100
        self.controller_ts = 1.0 / self.controller_rate

        # Mass command publishers
        self.pub_mass0 = rospy.Publisher(
            'movable_mass_0_position_controller/command', Float64, queue_size=1)
        self.pub_mass1 = rospy.Publisher(
            'movable_mass_1_position_controller/command', Float64, queue_size=1)
        self.pub_mass2 = rospy.Publisher(
            'movable_mass_2_position_controller/command', Float64, queue_size=1)
        self.pub_mass3 = rospy.Publisher(
            'movable_mass_3_position_controller/command', Float64, queue_size=1)

        self.mass0_command_msg = Float64()
        self.mass1_command_msg = Float64()
        self.mass2_command_msg = Float64()
        self.mass3_command_msg = Float64()

        # Referent angle subscriber
        self.angle_subscriber = rospy.Subscriber(
            "angle_ref",
            Vector3,
            self.angle_cb)

        # Imu subscriber
        self.imu_sub = rospy.Subscriber(
            'imu',
            Imu,
            self.ahrs_cb)

        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_rate_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.t_old = 0

        # Roll PID control
        self.pid_roll = PID(3, 1, 0)
        self.pid_roll_rate =PID(2.5, 0, 0)

        # Pitch PID control
        self.pid_pitch = PID(3, 1, 0)
        self.pid_pitch_rate = PID(2.5, 0, 0)

        # Roll compensator
        self.lambda_roll = 0.5
        self.pid_compensator_roll = PID(2 * self.lambda_roll, self.lambda_roll ** 2, 1)
        self.roll_compensator_gain = 0.05

        # Roll rate compensator
        self.lambda_roll_rate = 0.8
        self.pid_compensator_roll_rate = PID(2 * self.lambda_roll_rate, self.lambda_roll_rate ** 2, 1)
        self.roll_rate_compensator_gain = 0.5

        # Pitch compensator
        self.lambda_pitch = 0.5
        self.pid_compensator_pitch = PID(2 * self.lambda_pitch, self.lambda_pitch ** 2, 1)
        self.pitch_compensator_gain = 0.05

        # Pitch rate compensator
        self.lambda_pitch_rate = 0.8
        self.pid_compensator_pitch_rate = PID(2 * self.lambda_pitch_rate, self.lambda_pitch_rate ** 2, 1)
        self.pitch_rate_compensator_gain = 0.5

        # Define feed forward roll reference filter
        self.roll_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.roll_feed_forward_gain = 0.1

        # Define feed forward roll rate reference filter
        self.roll_rate_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.roll_rate_feed_forward_gain = 0.5

        # Define feed forward roll reference filter
        self.pitch_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.pitch_feed_forward_gain = 0.1

        # Define feed forward roll rate reference filter
        self.pitch_rate_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.pitch_rate_feed_forward_gain = 0.5

        self.pitch_acc_min = -100
        self.pitch_acc_max = 100

        self.roll_acc_min = -100
        self.roll_acc_max = 100

        self.roll_rate_min = -0.3
        self.roll_rate_max = 0.3

        self.pitch_rate_min = -0.3
        self.pitch_rate_max = 0.3

        self.eps = 0.01
        self.roll_beta = 0.01
        self.roll_rate_beta = 0.01
        self.pitch_beta = 0.01
        self.pitch_rate_beta = 0.01

    def angle_cb(self, data):

        self.euler_sp = Vector3(data.x, data.y, data.z)

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

        self.first_measurement = True

        self.p = data.twist.twist.angular.x
        self.q = data.twist.twist.angular.y
        self.r = data.twist.twist.angular.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w

    def ahrs_cb(self, msg):

        if not self.first_measurement:
            self.first_measurement = True

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = msg.angular_velocity.x
        q = msg.angular_velocity.y
        r = msg.angular_velocity.z

        sx = math.sin(self.euler_mv.x)   # sin(roll)
        cx = math.cos(self.euler_mv.x)   # cos(roll)
        cy = math.cos(self.euler_mv.y)   # cos(pitch)
        ty = math.tan(self.euler_mv.y)   # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r


    def run(self):
        """ Run ROS node - computes SMC algorithm for z and vz control """

        while not self.first_measurement:
            print("SmcAttitudeControl.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        print("SmcAttitudeControl.run() - Starting attitude control")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():

            # Sleep for controller rate
            rospy.sleep(self.controller_ts)

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            if dt < 1.0/self.controller_rate:
                continue

            # ROLL BLOCK
            roll_error = self.euler_sp.x - self.euler_mv.x
            self.euler_rate_sp.x = self.calculate_roll_rate_ref(dt, roll_error)

            # ROLL RATE BLOCK
            roll_rate_error = self.euler_rate_sp.x - self.euler_rate_mv.x
            mass_roll_offset = self.calculate_mass_offset_roll(dt,roll_rate_error)

            # PITCH BLOCK
            pitch_error = self.euler_sp.y - self.euler_mv.y
            self.euler_rate_sp.y = self.calculate_pitch_rate_ref(dt, pitch_error)

            # PITCH RATE BLOCK
            pitch_rate_error = self.euler_rate_sp.y - self.euler_rate_mv.y
            mass_pitch_offset = self.calculate_mass_offset_pitch(dt,pitch_rate_error)

            self.mass0_command_msg.data = mass_pitch_offset
            self.mass1_command_msg.data = -mass_roll_offset
            self.mass2_command_msg.data = -mass_pitch_offset
            self.mass3_command_msg.data = mass_roll_offset

            self.pub_mass0.publish(self.mass0_command_msg)
            self.pub_mass1.publish(self.mass1_command_msg)
            self.pub_mass2.publish(self.mass2_command_msg)
            self.pub_mass3.publish(self.mass3_command_msg)

    def calculate_roll_rate_ref(self, dt, roll_error):
        roll_pid_output = self.pid_roll.compute(roll_error, dt)
        roll_compensator_term = self.pid_compensator_roll.compute(roll_error, dt)
        roll_switch_term = self.roll_beta * math.tanh(roll_compensator_term / self.eps)
        roll_ff_term = self.roll_feed_forward_gain * self.roll_feed_forward_filter.compute(self.euler_sp.x)
        roll_ff_term = saturation(roll_ff_term, self.roll_rate_min, self.roll_rate_max)

        roll_rate_sp = \
            roll_pid_output  # + \
        # self.roll_compensator_gain * roll_compensator_term + \
        # roll_switch_term + \
        # roll_ff_term

        return roll_rate_sp

    def calculate_pitch_rate_ref(self, dt, pitch_error):
        pitch_pid_output = self.pid_pitch.compute(pitch_error, dt)
        pitch_compensator_term = self.pid_compensator_pitch.compute(pitch_error, dt)
        pitch_switch_term = self.pitch_beta * math.tanh(pitch_compensator_term / self.eps)
        pitch_ff_term = self.pitch_feed_forward_gain * self.pitch_feed_forward_filter.compute(self.euler_sp.y)
        pitch_ff_term = saturation(pitch_ff_term, self.pitch_rate_min, self.pitch_rate_max)

        pitch_rate_sp = \
            pitch_pid_output  # + \
        # self.pitch_compensator_gain * pitch_compensator_term + \
        # pitch_switch_term + \
        # pitch_ff_term

        return pitch_rate_sp

    def calculate_mass_offset_roll(self, dt, roll_rate_error):
        pid_roll_rate_term = self.pid_roll_rate.compute(roll_rate_error,dt)
        pid_comp_roll_rate = self.pid_compensator_roll_rate.compute(roll_rate_error,dt)
        pid_switch_roll_rate = self.roll_rate_beta * math.tanh(pid_comp_roll_rate/self.eps)
        roll_rate_ff_term = self.roll_rate_feed_forward_gain * self.roll_rate_feed_forward_filter.compute(
            self.euler_rate_sp.x)

        roll_offset = \
            pid_roll_rate_term #+ \
            #self.roll_rate_compensator_gain * pid_comp_roll_rate + \
            #pid_switch_roll_rate + \
            #roll_rate_ff_term

        return roll_offset

    def calculate_mass_offset_pitch(self, dt, pitch_rate_error):
        pid_pitch_rate_term = self.pid_pitch_rate.compute(pitch_rate_error,dt)
        pid_comp_pitch_rate = self.pid_compensator_pitch_rate.compute(pitch_rate_error,dt)
        pid_switch_pitch_rate = self.pitch_rate_beta * math.tanh(pid_comp_pitch_rate/self.eps)
        pitch_rate_ff_term = self.pitch_rate_feed_forward_gain * self.pitch_rate_feed_forward_filter.compute(
            self.euler_rate_sp.y)

        pitch_offset = \
            pid_pitch_rate_term #+ \
            #self.pitch_rate_compensator_gain * pid_comp_pitch_rate + \
            #pid_switch_pitch_rate + \
            #pitch_rate_ff_term

        return pitch_offset


if __name__ == "__main__":

    rospy.init_node('smc_attitude_control', anonymous=True)
    try:
        attitude_control = SmcAttitudeController()
        attitude_control.run()
    except rospy.ROSInterruptException:
        pass


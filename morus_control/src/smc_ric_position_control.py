#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from pid_smc import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import Float32, Int8
from dynamic_reconfigure.server import  Server
from dynamic_reconfigure.client import Client
from morus_msgs.cfg import SmcPosCtlParamsConfig
from morus_msgs.msg import PIDController
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
import math
from datetime import datetime
from morus_msgs.msg import SMCStatusPosition
from simple_filters import deadzone
from std_msgs.msg import Header


class SmcPositionControl:

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.x_sp = 0                   # x-position set point
        self.y_sp = 0                   # y-position set point
        self.x_mv = 0                   # x-position measured value
        self.y_mv = 0                   # y-position measured value
        self.pid_x = PID(0.5, 0, 0, 5, -5)                   # pid instance for x control
        self.pid_y = PID(0.5, 0, 0, 5, -5)                   # pid instance for y control

        self.vx_sp = 0                  # vx velocity set_point
        self.vy_sp = 0                  # vx velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.vy_mv = 0                  # vy velocity measured value
        self.vx_mv_old = 0              # vz velocity old measured value
        self.vy_mv_old = 0              # vz velocity old measured value
        self.pid_vx = PID(0.1, 0, 0, 0.02, -0.02)             # pid instance for x-velocity control
        self.pid_vy = PID(0.1, 0, 0, 0.02, -0.02)             # pid instance for y-velocity control

        self.euler_mv = Vector3(0, 0, 0)        # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)        # euler angles referent values
        self.euler_rate_mv = Vector3(0, 0, 0)   # measured angular velocities

        self.t_old = 0
        self.x_ref = 0
        self.y_ref = 0
        self.x_sp = 0
        self.y_sp = 0
        self.yaw_sp = 0

        self.filter_const_meas = 0.9
        self.filter_const_ref = 5.0

        rospy.Subscriber('position', PointStamped, self.pos_cb)
        rospy.Subscriber('velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('imu', Imu, self.ahrs_cb)

        self.euler_ref_pub = rospy.Publisher('angle_ref', Vector3, queue_size=1)
        self.status_pub = rospy.Publisher('smc_status_pos', SMCStatusPosition, queue_size=1)
        self.status_msg = SMCStatusPosition()

        self.rate = 100.0
        self.Ts = 1.0 / self.rate 
        self.ros_rate = rospy.Rate(self.rate)
        self.t_start = rospy.Time.now()

        self.update_filter_coefficients()
        rospy.loginfo("Initialized position controller.")

        # X - pos compensator
        self.lambda_x = 0.1
        self.pid_compensator_x = PID(2 * self.lambda_x, self.lambda_x ** 2, 0.0)
        self.x_compensator_gain = 0.1

        # Y - pos rate compensator
        self.lambda_y = 0.1
        self.pid_compensator_y = PID(2 * self.lambda_y, self.lambda_y ** 2, 0.5)
        self.y_compensator_gain = 0.001

        # VX compensator
        self.lambda_vx = 0.1
        self.pid_compensator_vx = PID(2 * self.lambda_vx, self.lambda_vx ** 2, 0.0)
        self.vx_compensator_gain = 0.001

        # VY compensator
        self.lambda_vy = 0.1
        self.pid_compensator_vy = PID(2 * self.lambda_vy, self.lambda_vy ** 2, 0.5)
        self.vy_compensator_gain = 0.001

        self.eps = 0.5
        self.vel_eps = 0.5

        self.x_beta = 0.01
        self.vx_beta = 0.001
        self.y_beta = 0.01
        self.vy_beta = 0.001

        self.cfg_server = Server(SmcPosCtlParamsConfig, self.cfg_callback)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag:
            # print 'Waiting for velocity measurements.'
            rospy.sleep(0.5)
        print "SMCPositionControl.run() - Starting position control."

        self.t_old = rospy.Time.now()
        while not rospy.is_shutdown():
            
            self.ros_rate.sleep()

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            if dt < 1.0 / self.rate:
                continue

            # Position prefilters
            self.x_ref = self.filter_ref_a * self.x_ref + (1.0 - self.filter_ref_a) * self.x_sp
            self.y_ref = self.filter_ref_a * self.y_ref + (1.0 - self.filter_ref_a) * self.y_sp

            x_error = self.x_ref - self.x_mv
            y_error = self.y_ref - self.y_mv

            x_error = deadzone(x_error, -0.01, 0.01)
            y_error = deadzone(y_error, -0.01, 0.01)

            # Outer loop - PID
            vx_ref = self.pid_x.compute(x_error, dt)
            vy_ref = self.pid_y.compute(y_error, dt)

            vx_error = vx_ref - self.vx_mv
            vy_error = vy_ref - self.vy_mv

            # Inner loop - SMC
            pitch_r = self.vx_inner_loop(vx_error, dt)
            roll_r = - self.vy_inner_loop(vy_error, dt)

            # Add correction with regard to current yaw angle
            roll_ref = math.cos(self.euler_mv.z) * roll_r + math.sin(self.euler_mv.z) * pitch_r
            pitch_ref = -math.sin(self.euler_mv.z) * roll_r + math.cos(self.euler_mv.z) * pitch_r

            # Publish referent value
            vec3_msg = Vector3(roll_ref, pitch_ref, 0)
            self.euler_ref_pub.publish(vec3_msg)

            # Status message
            head = Header()
            head.stamp = rospy.Time.now()
            self.status_msg.header = head

            self.status_msg.x_sp = self.x_ref
            self.status_msg.x_mv = self.x_mv
            self.status_msg.x_sp = self.y_ref
            self.status_msg.x_mv = self.y_mv

            self.status_msg.vx_sp = vx_ref
            self.status_msg.vx_mv = self.vx_mv
            self.status_msg.vx_sp = vy_ref
            self.status_msg.vx_mv = self.vy_mv

            self.status_pub.publish(self.status_msg)

    def vx_inner_loop(self, vx_error, dt):
        """
        Inner loop for x velocity control.

        :param vx_error:
        :param dt:
        :return: Pitch reference for attitude controller.
        """
        vx_pid_term = self.pid_vx.compute(vx_error, dt)
        vx_comp_term = self.pid_compensator_vy.compute(vx_error, dt)
        vx_switch_term = self.vx_beta * math.tanh(vx_comp_term / self.eps)

        # Update status message
        self.status_msg.vx_pid = vx_pid_term
        self.status_msg.vx_comp = self.vx_compensator_gain * vx_comp_term
        self.status_msg.vx_switch = vx_switch_term

        pitch_ref = + \
            vx_pid_term + \
            self.vx_compensator_gain * vx_comp_term + \
            vx_switch_term

        return pitch_ref

    def vy_inner_loop(self, vy_error, dt):
        """
        Inner loop for y velocity control.

        :param vy_error:
        :param dt:
        :return: Roll reference for attitude controller.
        """
        vy_pid_term = self.pid_vx.compute(vy_error, dt)
        vy_comp_term = self.pid_compensator_vy.compute(vy_error, dt)
        vy_switch_term = self.vy_beta * math.tanh(vy_comp_term / self.eps)

        # Update status message
        self.status_msg.vy_pid = vy_pid_term
        self.status_msg.vy_comp = self.vy_compensator_gain * vy_comp_term
        self.status_msg.vy_switch = vy_switch_term

        roll_ref = + \
            vy_pid_term + \
            self.vy_compensator_gain * vy_comp_term + \
            vy_switch_term

        return roll_ref

    def pos_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        self.x_mv = msg.point.x
        self.y_mv = msg.point.y
        self.z_mv = msg.point.z

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True

        self.vx_mv = self.filter_const_meas * self.vx_mv_old + (1 - self.filter_const_meas) * msg.twist.linear.x 
        self.vx_mv_old = self.vx_mv
        self.vy_mv = self.filter_const_meas * self.vy_mv_old + (1 - self.filter_const_meas) * msg.twist.linear.y 
        self.vy_mv_old = self.vy_mv

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vx_sp = msg.x
        self.vy_sp = msg.y

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.x_sp = msg.x
        self.y_sp = msg.y

    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
        '''
        #if not self.start_flag:
        #    self.start_flag = True

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

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def yaw_ref_cb(self, msg):
        '''
        Yaw ref  callback.
        :param msg: Type Float32 
        '''
        self.yaw_sp = msg.data

    def rotor_command_cb(self, msg):
        '''
        Vpc output callback, msg.x roll vpc out, msg.y pitch vpc out, msg.z yaw out
        :param msg: Type Vector3
        '''
        self.yaw_command = msg.z

    def update_filter_coefficients(self):
        self.filter_ref_a = self.filter_const_ref / (self.filter_const_ref + self.Ts)
        self.filter_ref_b = self.Ts / (self.filter_const_ref + self.Ts) 

        self.filter_meas_a = self.filter_const_meas / (self.filter_const_meas + self.Ts)
        self.filter_meas_b = self.Ts / (self.filter_const_meas + self.Ts) 

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for yaw and yaw rate controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server

            config.x_kp = self.pid_x.get_kp()
            config.x_ki = self.pid_x.get_ki()
            config.x_kd = self.pid_x.get_kd()

            config.vx_kp = self.pid_vx.get_kp()
            config.vx_ki = self.pid_vx.get_ki()
            config.vx_kd = self.pid_vx.get_kd()

            config.y_kp = self.pid_y.get_kp()
            config.y_ki = self.pid_y.get_ki()
            config.y_kd = self.pid_y.get_kd()

            config.vy_kp = self.pid_vy.get_kp()
            config.vy_ki = self.pid_vy.get_ki()
            config.vy_kd = self.pid_vy.get_kd()

            config.filter_ref = self.filter_const_ref
            config.filter_meas = self.filter_const_meas

            config.lambd = self.lambda_x
            config.vel_lambda = self.lambda_vx

            config.comp_gain = self.x_compensator_gain
            config.vel_comp_gain = self.vx_compensator_gain

            config.eps = self.eps
            config.vel_eps = self.vel_eps

            config.beta = self.x_beta
            config.vel_beta = self.vx_beta

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers

            self.pid_x.set_kp(config.x_kp)
            self.pid_x.set_ki(config.x_ki)
            self.pid_x.set_kd(config.x_kd)

            self.pid_vx.set_kp(config.vx_kp)
            self.pid_vx.set_ki(config.vx_ki)
            self.pid_vx.set_kd(config.vx_kd)

            self.pid_y.set_kp(config.y_kp)
            self.pid_y.set_ki(config.y_ki)
            self.pid_y.set_kd(config.y_kd)

            self.pid_vy.set_kp(config.vy_kp)
            self.pid_vy.set_ki(config.vy_ki)
            self.pid_vy.set_kd(config.vy_kd)

            self.filter_const_ref = config.filter_ref
            self.filter_const_meas = config.filter_meas
            self.update_filter_coefficients()

            self.pid_compensator_x.set_kp(2 * config.lambd)
            self.pid_compensator_x.set_ki(config.lambd ** 2)

            self.pid_compensator_y.set_kp(2 * config.lambd)
            self.pid_compensator_y.set_ki(config.lambd ** 2)

            self.pid_compensator_vx.set_kp(2 * config.vel_lambda)
            self.pid_compensator_vx.set_ki(config.vel_lambda ** 2)

            self.pid_compensator_vy.set_kp(2 * config.vel_lambda)
            self.pid_compensator_vy.set_ki(config.vel_lambda ** 2)

            self.x_compensator_gain = config.comp_gain
            self.y_compensator_gain = config.comp_gain

            self.vx_compensator_gain = config.vel_comp_gain
            self.vy_compensator_gain = config.vel_comp_gain

            self.eps = config.eps
            self.vel_eps = config.vel_eps

            self.x_beta = config.beta
            self.y_beta = config.beta

            self.vx_beta = config.vel_beta
            self.vy_beta = config.vel_beta

        # this callback should return config data back to server
        return config


if __name__ == '__main__':

    rospy.init_node('smc_pos_ctl')
    rospy.loginfo('Initializing position control')
    pos_ctl = SmcPositionControl()
    rospy.loginfo('Running position control')
    pos_ctl.run()


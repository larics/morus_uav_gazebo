#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from morus_msgs.msg import PIDController
from std_msgs.msg import *
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from pid import PID

VERBOSE = True



class AngleTiltCtl:

    def __init__(self):


        self.first_measurement = False
        self.t = 0
        self.t_old = 0

        # Initialize subscribers
        # Pose subscriber
        self.pose_sub = rospy.Subscriber(
            '/morus/pose',
            PoseStamped,
            self.pose_cb)
        # Odometry subscriber
        self.odometry = rospy.Subscriber(
            '/morus/odometry',
            Odometry,
            self.odometry_cb)
        # IMU subscriber
        self.imu = rospy.Subscriber(
            '/morus/Imu',
            Imu,
            self.imu_cb)
        # Pose reference subscriber
        self.pose_sp = rospy.Subscriber('/morus/pose_ref', Vector3, self.pose_sp_cb)
        self.angle_sp = rospy.Subscriber('/morus/angle_ref', Vector3, self.angle_sp_cb)
        self.tilt_x_sp = rospy.Subscriber('/morus/tilt_x_ref', Float64, self.tilt_sp_x_cb)
        self.tilt_y_sp = rospy.Subscriber('/morus/tilt_y_ref', Float64, self.tilt_sp_y_cb)
        self.ref_tilt = rospy.Subscriber('/morus/tilt_ref', Float64, self.ref_tilt_cb)
        self.vel_ref_sub = rospy.Subscriber('/morus/lin_vel_ref', Vector3, self.vel_ref_cb)

        # Initialize publishers, motor speed and tilt for roll and pitch
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.pub_roll_tilt0 = rospy.Publisher('/morus/angle_tilt_0_controller/command', Float64, queue_size=1)
        self.pub_roll_tilt1 = rospy.Publisher('/morus/angle_tilt_2_controller/command', Float64, queue_size=1)
        self.pub_pitch_tilt0 = rospy.Publisher('/morus/angle_tilt_1_controller/command', Float64, queue_size=1)
        self.pub_pitch_tilt1 = rospy.Publisher('/morus/angle_tilt_3_controller/command', Float64, queue_size=1)
        self.pub_tilt_ref = rospy.Publisher("/morus/tilt_ref", Float64, queue_size=1)
        self.pub_tilt_x_ref = rospy.Publisher("/morus/tilt_x_ref_", Float64, queue_size=1)
        self.pub_tilt_y_ref = rospy.Publisher("/morus/tilt_y_ref_", Float64, queue_size=1)
        self.pub_vel_ref = rospy.Publisher("/morus/pub_lin_vel_ref", Vector3, queue_size=1)

        # Publishing PIDs in order to use dynamic reconfigure
        self.pub_PID_z = rospy.Publisher('PID_z', PIDController, queue_size=1)
        self.pub_PID_vz = rospy.Publisher('PID_vz', PIDController, queue_size=1)
        self.pub_pitch_rate_PID = rospy.Publisher('PID_pitch_rate', PIDController, queue_size=1)
        self.pub_pitch_PID = rospy.Publisher('PID_pitch', PIDController, queue_size=1)
        self.pub_roll_rate_PID = rospy.Publisher('PID_roll_rate', PIDController, queue_size=1)
        self.pub_roll_PID = rospy.Publisher('PID_roll', PIDController, queue_size=1)
        self.pub_yaw_PID = rospy.Publisher('PID_yaw_rate_PID', PIDController, queue_size=1)
        self.pub_angles = rospy.Publisher('/morus/euler_mv', Vector3, queue_size=1)
        self.pub_angles_sp = rospy.Publisher('/morus/euler_sp', Vector3, queue_size=1)
        self.ros_rate = rospy.Rate(50)

        self.z_sp = 0                           # z-position set point
        self.z_ref_filt = 0                     # z ref filtered
        self.z_mv = 0                           # z-position measured value
        self.pid_z = PID()                      # pid instance for z control

        self.vz_sp = 0                          # vz velocity set_point
        self.vz_mv = 0                          # vz velocity measured value
        self.pid_vz = PID()                     # pid instance for z-velocity control

        self.euler_mv = Vector3(0., 0., 0)      # measured euler angles
        self.euler_sp = Vector3(0., 0., 0.)     # euler angles referent values
        self.pose_sp = Vector3(0., 0., 0.0)
        self.pose_mv = Vector3(0., 0., 0.)
        self.pose_mv_tf = Vector3(0., 0., 0.)
        self.vel_mv = Vector3(0., 0., 0.)
        self.vel_ref = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0, 0, 0)   # measured angular velocities
        self.roll_sp_filt, self.pitch_sp_filt, self.yaw_sp_filt = 0, 0, 0
        self.dwz = 0
        self.tilt_x = 0
        self.tilt_y = 0

        self.lim_tilt = 0.15
        c = 1000

        ########################################
        # Position control PIDs -> connect directly to rotors tilt
        self.pid_x = PID(1., 0, 0., self.lim_tilt, -self.lim_tilt)
        self.pid_vx = PID(1, 0.01, 0.1, 300, -300)
        self.pid_y = PID(1., 0, 0.0, self.lim_tilt, -self.lim_tilt)
        self.pid_vy = PID(1, 0.01, 0.1, 300, -300)

        # Define PID for height control
        self.z_ref_filt = 0
        self.z_mv = 0

        self.pid_z = PID(3, 0.01, 1.5, 4, -4)
        self.pid_vz = PID(85, 0.1, 15, 200, -200)

        ########################################
        ########################################

        # initialize pitch_rate PID controller
        self.pitch_rate_PID = PID(4, 0.05, 1, 50, -50)
        self.pitch_PID = PID(75, 0, 1, 50, -50)

        # initialize roll rate PID controller
        self.roll_rate_PID = PID(4, 0.05, 1, 50, -50)
        self.roll_PID = PID(75, 0, 1, 50, -50)

        # initialize yaw rate PID controller
        self.yaw_rate_PID = PID(15, 0.1, 25, 5, -5)
        self.yaw_PID = PID(95, 0.5, 65, +150, -150)

    def ref_tilt_cb(self, msg):
        self.ref_tilt = msg.data

    def vel_ref_cb(self, msg):
        self.vel_ref.x = msg.x
        self.vel_ref.y = msg.y
        self.vel_ref.z = msg.z

    def pose_cb(self, msg):
        """
        Callback functon for assigning values from pose IMU

        :param msg: /morus/pose PoseStamped msg type used to extract pose and orientation of UAV

        """
        # entered subscribed callback func, set first_measurement flag as True
        self.first_measurement = True

        self.pose_mv.x = msg.pose.position.x
        self.pose_mv.y = msg.pose.position.y
        self.pose_mv.z = msg.pose.position.z

        self.pose_mv_tf.x = msg.pose.position.x * np.cos(self.euler_sp.z) + msg.pose.position.y * np.sin(self.euler_sp.z)
        self.pose_mv_tf.y = - msg.pose.position.x * np.sin(self.euler_sp.z) + msg.pose.position.y * np.cos(self.euler_sp.z)

        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w

    def tilt_sp_x_cb(self, msg):
        self.tilt_x = msg.data

    def tilt_sp_y_cb(self, msg):
        self.tilt_y = msg.data


    def odometry_cb(self, msg):
        """
        Callback function for assigning values from odometry measurements

        :param msg: nav_msgs/Odometry,
        an estimate of position and velocity in free space
        """

        self.vel_mv.x = msg.twist.twist.linear.x
        self.vel_mv.y = msg.twist.twist.linear.y
        self.vel_mv.z = msg.twist.twist.linear.z
        # TO DO: transform speeds global speed

        self.euler_rate_mv.x = msg.twist.twist.angular.x
        self.euler_rate_mv.y = msg.twist.twist.angular.y
        self.euler_rate_mv.z = msg.twist.twist.angular.z

    def pose_sp_cb(self, msg):

        self.pose_sp.x = msg.x
        self.pose_sp.y = msg.y
        self.pose_sp.z = msg.z

    def angle_sp_cb(self, msg):

        self.euler_sp.x = msg.x
        self.euler_sp.y = msg.y
        self.euler_sp.z = msg.z

    def imu_cb(self, msg):
        """
        Callback function used to extract measured values from IMU, lin_acc and ang_vel
        :param msg:
        :return:
        """

        self.lin_acc_x = msg.linear_acceleration.x
        ## TO DO: add rest if needed

    def quat_to_eul_conv(self, qx, qy, qz, qw):
        """
        Convert quaternions to euler angles (roll, pitch, yaw)
        """

        # roll (x-axis rotation)
        sinr = 2. * (qw * qx +  qy * qz)
        cosr = 1. - 2. * (qx * qx + qy * qy)
        self.roll = math.atan2(sinr, cosr)

        # pitch (y-axis rotation)
        sinp = 2. * (qw * qy - qz * qx)
        sinp = 1. if sinp > 1. else sinp
        sinp = -1. if sinp < -1. else sinp
        self.pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny = 2. * (qw * qz + qx * qy)
        cosy = 1 - 2. * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny, cosy)

        self.euler_mv.x = self.roll
        self.euler_mv.y = self.pitch
        self.euler_mv.z = self.yaw

    def run(self):
        """
        Runs quadcopter control algorithm
        """

        while not self.first_measurement:
            self.tilt_x = 0
            self.tilt_y = 0
            self.ref_tilt = 0
            self.pub_tilt_x_ref.publish(self.tilt_x)
            self.pub_tilt_y_ref.publish(self.tilt_y)

            print("Waiting for first measurement")
            rospy.sleep(0.5)
        print("Started angle control")
        self.t_old = rospy.Time.now()


        while not rospy.is_shutdown():
            self.ros_rate.sleep()
            # discretization time
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            if dt > 0.105 or dt < 0.095:
                print(dt)
            if dt < 0.01:
                dt = 0.01

            self.t_old = t
            self.quat_to_eul_conv(self.qx, self.qy, self.qz, self.qw)

            self.hover_speed = math.sqrt(293/0.000456874/4)

            # HEIGHT CONTROL:
            a = 0.2
            self.z_ref_filt = (1 - a) * self.z_ref_filt + a * self.pose_sp.z
            vz_ref = self.pid_z.compute(self.z_ref_filt, self.pose_mv.z, dt)
            domega_z = self.pid_vz.compute(vz_ref, self.vz_mv, dt)

            ## ATTITUDE CONTROL:

            # roll cascade (first PID -> y ref, second PID -> tilt_roll_rate ref)
            #self.euler_sp.x = - self.pid_y.compute(self.pose_sp.y, self.pose_mv.y, dt)
            b = 0.2
            self.roll_sp_filt =  (1 - b) * self.euler_sp.x + b * self.roll_sp_filt
            roll_rate_ref = self.roll_rate_PID.compute(self.euler_sp.x, self.euler_mv.x, dt)
            dwx = self.roll_PID.compute(roll_rate_ref, self.euler_rate_mv.x, dt)

            # pitch cascade(first PID -> pitch ref, second PID -> pitch_rate ref)
            #self.euler_sp.y = self.pid_x.compute(self.pose_sp.x, self.pose_mv.x, dt)
            b = 0.3
            self.pitch_sp_filt = (1 - b) * self.euler_sp.y + b * self.pitch_sp_filt
            pitch_rate_ref = self.pitch_rate_PID.compute(self.pitch_sp_filt, self.euler_mv.y, dt)
            dwy = self.pitch_PID.compute(pitch_rate_ref, self.euler_rate_mv.y, dt)
            #dwy = 0
            # yaw cascade (first PID -> yaw ref, second PID -> yaw_rate ref)
            a = 0.8
            self.yaw_sp_filt = (1 - a) * self.euler_sp.z + a * self.yaw_sp_filt
            yaw_rate_ref = self.yaw_rate_PID.compute(self.yaw_sp_filt, self.euler_mv.z, dt)
            dwz = self.yaw_PID.compute(yaw_rate_ref, self.euler_rate_mv.z, dt)

            # pose control with rotors tilt


            # NE ODKOMENTIRAVAT
            # Global pose -> local pose
            #pose_x_corr = self.pose_sp.x * np.cos(self.yaw) + self.pose_sp.y * np.sin(self.yaw)
            #pose_y_corr = - self.pose_sp.x * np.sin(self.yaw) + self.pose_sp.y * np.cos(self.yaw)

            # Global speed -> local speed
            vel_mv_x_corr = self.vel_mv.x * np.cos(self.yaw) - self.vel_mv.y * np.sin(self.yaw)
            vel_mv_y_corr = self.vel_mv.x * np.sin(self.yaw) + self.vel_mv.y * np.cos(self.yaw)

            vel_mv_x_corr = self.vel_mv.x
            vel_mv_y_corr = self.vel_mv.y

            pose_sp_x = prefilter(self.pose_mv.x, 0.5, self.pose_sp.x)
            pose_sp_y = prefilter(self.pose_mv.y, 0.5, self.pose_sp.y)

            pose_sp_tf_x = np.cos(self.euler_sp.z) * pose_sp_x + np.sin(self.euler_sp.z) * pose_sp_y
            pose_sp_tf_y = - np.sin(self.euler_sp.z) * pose_sp_x + np.cos(self.euler_sp.z) * pose_sp_y

            vel_sp_x = self.pid_vx.compute(pose_sp_tf_x, self.pose_mv_tf.x, dt)
            vel_sp_y = self.pid_vy.compute(pose_sp_tf_y, self.pose_mv_tf.y, dt)

            # --> added vel_ref to configure inner control loop
            tilt_x = self.pid_x.compute(self.vel_ref.x, self.vel_mv.x, dt)
            tilt_y = self.pid_y.compute(self.vel_ref.y, self.vel_mv.y, dt)

            tilt_tf_x = tilt_x
            tilt_tf_y = tilt_y

            # small addition in order to be still while on ground
            if self.pose_mv.z < 1.0:
                tilt_tf_x = 0
                tilt_tf_y = 0
            # NE ODKOMENTIRAVAT
            #tilt_tf_x = np.cos(self.yaw) * tilt_x - np.sin(self.yaw) * tilt_y
            #tilt_tf_y = np.sin(self.yaw) * tilt_y + np.cos(self.yaw) * tilt_y

            if VERBOSE:

                print("Pitch speed: {}\n Roll speed: {}\n, Yaw speed: {}\n".format(dwy, dwx, dwz))
                print("Yaw measured_value:{}\n Yaw_reference_value:{}\n".format(self.euler_mv.z, self.euler_sp.z))
                print("Roll measured_value:{}\n, Roll_reference_value:{}\n".format(self.euler_mv.x, self.euler_sp.x))
                print("Pitch measured_value:{}\n, Pitch_reference_value:{}\n".format(self.euler_mv.y, self.euler_sp.y))
                print("x_m:{}\nx:{}\ny_m:{}\ny:{}\nz_m:{}\nz:{}".format(self.pose_mv.x, self.pose_sp.x,
                                                                        self.pose_mv.y, self.pose_sp.y,
                                                                        self.pose_mv.z,  self.pose_sp.z))
            #self.hover_speed = 0
            motor_speed_1 = self.hover_speed + domega_z + dwz - dwy
            motor_speed_2 = self.hover_speed + domega_z - dwz + dwx
            motor_speed_3 = self.hover_speed + domega_z + dwz + dwy
            motor_speed_4 = self.hover_speed + domega_z - dwz - dwx
            print("rotor_front:{}\nrotor_left:{}\nrotor_back:{}\nrotor_right:{}\n".format(motor_speed_1,
                                                                                          motor_speed_2,
                                                                                          motor_speed_3,
                                                                                          motor_speed_4))
            pose_error_quad = (abs(self.pose_sp.x - self.pose_mv.x) + abs(self.pose_sp.y - self.pose_mv.y)) ** 2
            #if pose_error_quad < 0.5:
            #    if abs(tilt_tf_x) > 0.1:
            #        tilt_tf_x = np.sign(tilt_tf_x) * 0.02
            #    if abs(tilt_tf_y) > 0.1:
            #        tilt_tf_y = np.sign(tilt_tf_y) * 0.02

            print("tilt_x output:{}\ntilt_y output:{}\n".format(tilt_tf_x, tilt_tf_y))
            print("=========GLOBAL==========\n")
            print("pose_sp.x:{}\npose_mv.x:{}\n").format(self.pose_sp.x, self.pose_mv.x)
            print("pose_sp.y:{}\npose_mv.y:{}\n").format(self.pose_sp.y, self.pose_mv.y)
            print("=========LOCAL=========\n")
            print("pose_sp.y:{}\npose_mv.y:{}\n".format(pose_sp_tf_y, self.pose_mv_tf.y))
            print("pose_sp.x:{}\npose_mv.x:{}\n".format(pose_sp_tf_x, self.pose_mv_tf.x))
            print("vel_mv.x:{}\nvel_mv.y:{}\n".format(self.vel_mv.x, self.vel_mv.y))
            print("yaw value: {}\n".format(self.euler_mv.z))
            print("Yaw output: {}\n".format(dwz))
            print("Roll control activated: {}".format(tilt_tf_y))
            print("Pitch control activated: {}".format(tilt_tf_x))
            print("Quadratic pose error is:{}\n".format(pose_error_quad))
            print("vel_ref_x:{}\nvel_ref_y:{}\ntilt_x_out:{}\ntilt_y_out:{}\n".format(
                  self.vel_ref.x, self.vel_ref.y, tilt_x, tilt_y))

            # CONTROL TILT
            self.pub_roll_tilt0.publish(-tilt_tf_y)
            self.pub_roll_tilt1.publish(+tilt_tf_y)
            self.pub_pitch_tilt0.publish(tilt_tf_x)
            self.pub_pitch_tilt1.publish(-tilt_tf_x)

            # PLOT TILT
            #self.pub_roll_tilt0.publish(self.tilt_y)
            #self.pub_roll_tilt1.publish(-self.tilt_y)
            #self.pub_pitch_tilt0.publish(self.tilt_x)
            #self.pub_pitch_tilt1.publish(-self.tilt_x)

            #print("TILT X is: {}".format(self.tilt_x))
            #new_tilt = - self.tilt_x
            #self.pub_pitch_tilt0.publish(self.tilt_x)
            #self.pub_pitch_tilt1.publish(new_tilt)
            motor_speed_msg = Actuators()
            motor_speed_msg.angular_velocities = [motor_speed_1, motor_speed_2,
                                                  motor_speed_3, motor_speed_4]

            # publish PID data -> could be useful for tuning
            #self.pub_PID_z.publish(self.pid_z.create_msg())
            #self.pub_PID_vz.publish(self.pid_vz.create_msg())
            #self.pub_pitch_PID.publish(self.pitch_PID.create_msg())
            #self.pub_pitch_rate_PID.publish(self.pitch_rate_PID.create_msg())
            #self.pub_roll_PID.publish(self.roll_PID.create_msg())
            #self.pub_roll_rate_PID.publish(self.roll_rate_PID.create_msg())
            #self.pub_yaw_PID.publish(self.yaw_PID.create_msg())
            # publish reference_data
            #self.pub_tilt_x_ref.publish(self.tilt_x)
            #self.pub_tilt_y_ref.publish(self.tilt_y)


            # publish_angles
            self.pub_angles.publish(self.euler_mv)
            self.pub_angles_sp.publish(self.euler_sp)
            self.pub_vel_ref.publish(self.vel_ref)

            self.pub_mot.publish(motor_speed_msg)


def prefilter(start_val,  coeff_val, setpoint_val):

    return (1 - coeff_val) * setpoint_val + coeff_val * start_val


if __name__ == "__main__":
    rospy.init_node('morus_angle_tilt_ctl')
    angle_ctl = AngleTiltCtl()
    angle_ctl.run()

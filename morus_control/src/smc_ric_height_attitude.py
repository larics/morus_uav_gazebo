#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pid import PID
from trajectory_msgs.msg import MultiDOFJointTrajectory
from first_order_filter import FirstOrderFilter

class MorusController:

    def __init__(self):
        """Constructor initializes all needed variables"""
        self.sleep_sec = 1
        self.first_measurement = False

        # Referent position subscriber
        self.pose_subscriber = rospy.Subscriber(
            "pos_ref",
            Vector3,
            self.setpoint_cb)
        # Referent angle subscriber
        self.angle_subscriber = rospy.Subscriber(
            "angle_ref",
            Vector3,
            self.angle_cb)

        # Odometry measurements subscriber
        self.odom_subscriber = rospy.Subscriber(
            "odometry",
            Odometry,
            self.odometry_callback)

        # initialize publishers
        self.motor_pub = rospy.Publisher(
            '/gazebo/command/motor_speed',
            Actuators,
            queue_size=10)
        self.motor_vel_msg = Actuators()

        # Mass command publishers
        self.pub_mass0 = rospy.Publisher(
            'morus/movable_mass_0_position_controller/command', Float64, queue_size=1)
        self.pub_mass1 = rospy.Publisher(
            'morus/movable_mass_1_position_controller/command', Float64, queue_size=1)
        self.pub_mass2 = rospy.Publisher(
            'morus/movable_mass_2_position_controller/command', Float64, queue_size=1)
        self.pub_mass3 = rospy.Publisher(
            'morus/movable_mass_3_position_controller/command', Float64, queue_size=1)

        self.pose_sp = Vector3(0., 0., 0.)
        self.vel_sp = Vector3(0., 0., 0.)
        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)
        self.t_old = 0

        # define PID for height control
        self.z_mv = 0

        # Controller rate
        self.controller_rate = 100
        self.rate = rospy.Rate(self.controller_rate)

        # Height controller
        # TODO: Implement real derivative in PID
        self.pid_z = PID(4, 0, 1)
        self.pid_vz = PID(75, 20, 10)

        # Z error compensator
        self.z_compensator = FirstOrderFilter(0.3, -0.299, 1)
        self.z_compensator_gain = 0.05

        # VZ error compensator
        self.vz_compensator = FirstOrderFilter(1, -0.998, 1)
        self.vz_compensator_gain = 100

        # Define switch function constants
        self.eps = 0.01
        self.z_pos_beta = 0.01
        self.vz_beta = 180

        # Define feed forward z position reference filter
        self.z_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.z_feed_forward_gain = 1

        # Define feed forward z velocity reference filter
        self.vz_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.vz_feed_forward_gain = 8

        # Define saturation limits
        self.vz_ref_min = -5
        self.vz_ref_max = 5

        self.rotor_vel_min = -400
        self.rotor_vel_max = 400
        self.hover_speed = 432.4305

    def setpoint_cb(self, data):

        self.pose_sp.x = data.x
        self.pose_sp.y = data.y
        self.pose_sp.z = data.z

    def linvel_cb(self, data):

        self.linvel_x = data.x
        self.linvel_y = data.y
        self.linvel_z = data.z

    def angle_cb(self, data):

        self.euler_sp = Vector3(data.x, data.y, data.z)

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

        self.first_measurement = True

        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vy_mv = data.twist.twist.linear.y
        self.vz_mv = data.twist.twist.linear.z

        self.p = data.twist.twist.angular.x
        self.q = data.twist.twist.angular.y
        self.r = data.twist.twist.angular.z

        self.qx = data.pose.pose.orientation.x
        self.qy = data.pose.pose.orientation.y
        self.qz = data.pose.pose.orientation.z
        self.qw = data.pose.pose.orientation.w

    def odometry_gt_callback(self, data):
        self.x_gt_mv = data.pose.pose.position.x
        self.y_gt_mv = data.pose.pose.position.y
        self.z_gt_mv = data.pose.pose.position.z

    def convert_to_euler(self, qx, qy, qz, qw):
        """Calculate roll, pitch and yaw angles/rates with quaternions"""

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler_mv.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw
                                     - qx * qx - qy * qy + qz * qz)
        self.euler_mv.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler_mv.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw
                                     + qx * qx - qy * qy - qz * qz)

        # gyro measurements (p,q,r)
        p = self.p
        q = self.q
        r = self.r

        sx = math.sin(self.euler_mv.x)  # sin(roll)
        cx = math.cos(self.euler_mv.x)  # cos(roll)
        cy = math.cos(self.euler_mv.y)  # cos(pitch)
        ty = math.tan(self.euler_mv.y)  # cos(pitch)

        # conversion gyro measurements to roll_rate, pitch_rate, yaw_rate
        self.euler_rate_mv.x = p + sx * ty * q + cx * ty * r
        self.euler_rate_mv.y = cx * q - sx * r
        self.euler_rate_mv.z = sx / cy * q + cx / cy * r

    def run(self):
        """ Run ROS node - computes SMC algorithm for z and vz control """

        while not self.first_measurement:
            print("MorusControl.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        print("MorusControl.run() - Starting position control")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            if dt < 1.0/self.controller_rate:
                continue

            # Calculate roll, pitch, yaw
            self.convert_to_euler(self.qx, self.qy, self.qz, self.qw)

            # Z POSITION BLOCK
            z_error = self.pose_sp.z - self.z_mv
            self.vel_sp.z = self.calculate_height_velocity_ref(dt, z_error)

            # Z VELOCITY BLOCK
            vz_error = self.vel_sp.z - self.vz_mv
            vz_error = saturation(vz_error, self.vz_ref_min, self.vz_ref_max)
            rotor_velocity = self.calculate_rotor_velocities(dt, vz_error)
            rotor_velocity = saturation(rotor_velocity, self.rotor_vel_min, self.rotor_vel_max)

            # Construct rotor velocity message
            self.motor_vel_msg.angular_velocities = \
                [
                    self.hover_speed + rotor_velocity,
                    self.hover_speed + rotor_velocity,
                    self.hover_speed + rotor_velocity,
                    self.hover_speed + rotor_velocity
                ]
            self.motor_pub.publish(self.motor_vel_msg)

    def calculate_height_velocity_ref(self, dt, z_error):

        # Calculate z position PID output
        z_pid_output = self.pid_z.compute(z_error, dt)

        # Calculate z position error compensator output
        z_error = self.pose_sp.z - self.z_mv
        z_error_compensator_term = self.z_compensator.compute(z_error)

        # Calculate z position switch function output
        z_pos_switch_output = self.z_pos_beta * math.tanh(z_error_compensator_term / self.eps)

        # Calculate feed-forward term from z position reference
        z_ref_ff_term = self.z_feed_forward_gain * self.z_feed_forward_filter.compute(self.pose_sp.z)

        # Calculate z velocity reference
        vel_ref = \
            z_pid_output + \
            self.z_compensator_gain * z_error_compensator_term + \
            z_pos_switch_output + \
            z_ref_ff_term

        return vel_ref

    def calculate_rotor_velocities(self, dt, vz_error):

        # Calculate z velocity PID output
        vz_pid_output = self.pid_vz.compute(vz_error, dt)

        # Calculate feed forward term from z velocity reference
        vz_ref_ff_term = self.vz_feed_forward_gain * self.vz_feed_forward_filter.compute(self.vel_sp.z)

        # Calculate z velocity error compensator output
        vz_error_compensator_term = self.vz_compensator.compute(vz_error)

        # Calculate velocity switch function output
        vz_switch_output = self.vz_beta * math.tanh(vz_error_compensator_term / self.eps)

        # Calculate rotor velocity
        rotor_velocity = \
            vz_ref_ff_term + \
            vz_pid_output + \
            self.vz_compensator_gain * vz_error_compensator_term + \
            vz_switch_output

        return rotor_velocity


def saturation(value, low, high):
    if value > high:
        return high
    elif value < low:
        return low
    else:
        return value


if __name__ == "__main__":
    rospy.init_node('morus_control', anonymous=True)
    try:
        morus_control = MorusController()
        morus_control.run()
    except rospy.ROSInterruptException:
        pass

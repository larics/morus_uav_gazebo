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
from morus_msgs.msg import SMCStatusHeight
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from morus_control.cfg import SmcUavHeightCtlParamsConfig
from nonlinear_blocks import deadzone, saturation
from sensor_msgs.msg import Imu

class SmcHeightController:

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

        # initialize publishers
        self.motor_pub = rospy.Publisher(
            '/gazebo/command/motor_speed',
            Actuators,
            queue_size=10)
        self.motor_vel_msg = Actuators()

        # Odometry measurements subscriber
        self.odom_subscriber = rospy.Subscriber(
            "odometry",
            Odometry,
            self.odometry_callback)

        # Imu subscriber
        self.imu_sub = rospy.Subscriber(
            'imu',
            Imu,
            self.ahrs_cb)

        # Status message publisher
        self.status_pub = rospy.Publisher(
            'smc_status_height',
            SMCStatusHeight,
            queue_size=1)
        self.status_msg = SMCStatusHeight()

        self.euler_sp = Vector3(0., 0., 0.)
        self.euler_rate_sp = Vector3(0., 0., 0.)
        self.euler_mv = Vector3(0., 0., 0.)
        self.euler_rate_mv = Vector3(0., 0., 0.)

        self.pose_sp = Vector3(0., 0., 1)
        self.vel_sp = Vector3(0., 0., 0.)
        self.t_old = 0

        # define PID for height control
        self.z_mv = 0

        # Controller rate
        self.controller_rate = 100
        self.controller_ts = 1.0 / self.controller_rate

        # Z PID Control
        self.pid_z = PID(4, 0, 1)

        # VZ PID Control
        self.pid_vz = PID(40, 0.1, 0)
        
        # Yaw PID Control
        self.pid_yaw = PID(4, 0, 7)
        self.pid_yaw_rate = PID(100, 0, 0)
        
        # Z compensator
        self.lambda_z = 0.1
        self.pid_compensator_z = PID(2 * self.lambda_z, self.lambda_z ** 2, 1)
        self.z_compensator_gain = 0.1
        
        # VZ Compensator
        self.lambda_vz = 2
        self.pid_compensator_vz = PID(2 * self.lambda_vz, self.lambda_vz ** 2, 0)
        self.vz_compensator_gain = 35 #100
        
        # Yaw compensator
        self.lambda_yaw = 0.1
        self.pid_compensator_yaw = PID(2 * self.lambda_yaw, self.lambda_yaw ** 2, 1)
        self.yaw_compensator_gain = 1
        
        # Yaw rate compensator
        self.lambda_yaw_rate = 0.1
        self.pid_compensator_yaw_rate = PID(2 * self.lambda_yaw_rate, self.lambda_yaw_rate ** 2, 0)
        self.yaw_rate_compensator_gain = 1
        
        # Define switch function constants
        self.eps = 0.01
        self.z_pos_beta = 0.01
        self.vz_beta = 50 # 180
        
        self.yaw_beta = 0.01
        self.yaw_rate_beta = 50 # 180
        
        # Define feed forward z position reference filter
        self.z_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.z_feed_forward_gain = 0.1

        # Define feed forward z velocity reference filter
        self.vz_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.vz_feed_forward_gain = 0.5
    
        # Yaw feed forward
        self.yaw_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.yaw_feed_forward_gain = 0.1
        
        # Yaw rate feed forward
        self.yaw_rate_feed_forward_filter = FirstOrderFilter(100, -100, 0.9048)
        self.yaw_rate_feed_forward_gain = 0.1
        
        # Define saturation limits
        self.vz_ref_min = -5
        self.vz_ref_max = 5

        self.rotor_vel_min = -400
        self.rotor_vel_max = 400
        self.hover_speed = 432.4305
        
        self.config_start = False
        self.cfg_server = Server(SmcUavHeightCtlParamsConfig, self.cfg_callback)

        # Z error compensator
        # self.z_compensator = FirstOrderFilter(0.3, -0.299, 1)
        # self.z_compensator_gain = 0.05

        # VZ error compensator
        # self.vz_compensator = FirstOrderFilter(1, -0.998, 1)
        # self.vz_compensator_gain = 100

    def cfg_callback(self, config, level):

        if not self.config_start:
            self.config_start = True

            # Height
            config.z_kp = self.pid_z.get_kp()
            config.z_ki = self.pid_z.get_ki()
            config.z_kd = self.pid_z.get_kd()

            config.vz_kp = self.pid_vz.get_kp()
            config.vz_ki = self.pid_vz.get_ki()
            config.vz_kd = self.pid_vz.get_kd()

            config.z_lambda = self.lambda_z
            config.z_comp_gain = self.z_compensator_gain

            config.vz_lambda = self.lambda_vz
            config.vz_comp_gain = self.vz_compensator_gain

            config.switch_eps = self.eps
            config.z_beta = self.z_pos_beta
            config.vz_beta = self.vz_beta

            config.z_ff = self.z_feed_forward_gain
            config.vz_ff = self.vz_feed_forward_gain

            # Yaw
            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_rate_kp = self.pid_yaw_rate.get_kp()
            config.yaw_rate_ki = self.pid_yaw_rate.get_ki()
            config.yaw_rate_kd = self.pid_yaw_rate.get_kd()

            config.yaw_lambda = self.lambda_yaw
            config.yaw_rate_lambda = self.lambda_yaw_rate

            config.yaw_comp_gain = self.yaw_compensator_gain
            config.yaw_rate_comp_gain = self.yaw_rate_compensator_gain

            config.yaw_beta = self.yaw_beta
            config.yaw_rate_beta = self.yaw_rate_beta

            config.yaw_ff = self.yaw_feed_forward_gain
            config.yaw_rate_ff = self.yaw_rate_feed_forward_gain

        else:
            # Height
            self.pid_z.set_kp(config.z_kp)
            self.pid_z.set_ki(config.z_ki)
            self.pid_z.set_kd(config.z_kd)

            self.pid_vz.set_kp(config.vz_kp)
            self.pid_vz.set_ki(config.vz_ki)
            self.pid_vz.set_kd(config.vz_kd)

            self.z_compensator_gain = config.z_comp_gain
            self.pid_compensator_z.set_kp(2 * config.z_lambda)
            self.pid_compensator_z.set_ki(config.z_lambda ** 2)
            # self.pid_compensator_z.set_kd(config.z_comp_kd)

            self.vz_compensator_gain = config.vz_comp_gain
            self.pid_compensator_vz.set_kp(2 * config.vz_lambda)
            self.pid_compensator_vz.set_ki(config.vz_lambda ** 2)
            # self.pid_compensator_vz.set_kd(config.vz_comp_kd)

            self.eps = config.switch_eps
            self.z_pos_beta = config.z_beta
            self.vz_beta = config.vz_beta

            self.z_feed_forward_gain = config.z_ff
            self.vz_feed_forward_gain = config.vz_ff

            # Yaw
            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_rate_kp)
            self.pid_yaw_rate.set_ki(config.yaw_rate_ki)
            self.pid_yaw_rate.set_kd(config.yaw_rate_kd)

            self.yaw_compensator_gain = config.yaw_comp_gain
            self.pid_compensator_yaw.set_kp(2 * config.yaw_lambda)
            self.pid_compensator_yaw.set_ki(config.yaw_lambda ** 2)

            self.yaw_rate_compensator_gain = config.yaw_rate_comp_gain
            self.pid_compensator_yaw_rate.set_kp(2 * config.yaw_rate_lambda)
            self.pid_compensator_yaw_rate.set_ki(config.yaw_rate_lambda ** 2)

            self.yaw_beta = config.yaw_beta
            self.yaw_rate_beta = config.yaw_rate_beta

            self.yaw_feed_forward_gain = config.yaw_ff
            self.yaw_rate_feed_forward_gain = config.yaw_rate_ff

        return config

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

    def angle_cb(self, data):

        self.euler_sp = Vector3(data.x, data.y, data.z)

    def setpoint_cb(self, data):

        self.pose_sp.x = data.x
        self.pose_sp.y = data.y
        self.pose_sp.z = data.z

    def linvel_cb(self, data):

        self.linvel_x = data.x
        self.linvel_y = data.y
        self.linvel_z = data.z

    def odometry_callback(self, data):
        """Callback function for odometry subscriber"""

        self.first_measurement = True

        self.x_mv = data.pose.pose.position.x
        self.y_mv = data.pose.pose.position.y
        self.z_mv = data.pose.pose.position.z

        self.vx_mv = data.twist.twist.linear.x
        self.vy_mv = data.twist.twist.linear.y
        self.vz_mv = data.twist.twist.linear.z

    def run(self):
        """ Run ROS node - computes SMC algorithm for z and vz control """

        while not self.first_measurement:
            print("SmcHeightControl.run() - Waiting for first measurement.")
            rospy.sleep(self.sleep_sec)

        print("SmcHeightControl.run() - Starting height control")
        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():

            # Sleep for controller rate
            rospy.sleep(self.controller_ts)

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            if dt < 1.0/self.controller_rate:
                continue

            # Z POSITION BLOCK
            z_error = self.pose_sp.z - self.z_mv
            self.vel_sp.z = self.calculate_height_velocity_ref(dt, z_error)

            # Z VELOCITY BLOCK
            vz_error = self.vel_sp.z - self.vz_mv
            vz_error = saturation(vz_error, self.vz_ref_min, self.vz_ref_max)
            rotor_velocity = self.calculate_rotor_velocities(dt, vz_error)
            rotor_velocity = saturation(rotor_velocity, self.rotor_vel_min, self.rotor_vel_max)

            # YAW BLOCK
            yaw_error = self.euler_sp.z - self.euler_mv.z
            self.euler_rate_sp.z = self.calculate_yaw_rate_ref(dt, yaw_error)
            self.euler_rate_sp.z = saturation(self.euler_rate_sp.z, -0.5, 0.5)

            # YAW RATE BLOCK
            yaw_rate_error = self.euler_rate_sp.z - self.euler_rate_mv.z
            yaw_rotor_velocity = self.calculate_yaw_rotor_velocity(dt, yaw_rate_error)
            yaw_rotor_velocity = saturation(yaw_rotor_velocity, -200, 200)

            # Construct rotor velocity message
            self.motor_vel_msg.angular_velocities = \
                [
                    self.hover_speed + rotor_velocity + yaw_rotor_velocity,
                    self.hover_speed + rotor_velocity - yaw_rotor_velocity,
                    self.hover_speed + rotor_velocity + yaw_rotor_velocity,
                    self.hover_speed + rotor_velocity - yaw_rotor_velocity
                ]
            self.motor_pub.publish(self.motor_vel_msg)

            # Update status message
            head = Header()
            head.stamp = rospy.Time.now()
            self.status_msg.header = head
            self.status_msg.z_mv = self.z_mv
            self.status_msg.z_sp = self.pose_sp.z
            self.status_msg.vz_sp = self.vel_sp.z
            self.status_msg.vz_mv = self.vz_mv
            self.status_msg.rotor_vel = self.hover_speed + rotor_velocity

            self.status_msg.yaw_mv = self.euler_mv.z
            self.status_msg.yaw_sp = self.euler_sp.z
            self.status_msg.yaw_rate_mv = self.euler_rate_mv.z
            self.status_msg.yaw_rate_sp = self.euler_rate_sp.z
            self.status_msg.yaw_rotor_vel = yaw_rotor_velocity

            # Publish status message
            self.status_pub.publish(self.status_msg)

    def calculate_height_velocity_ref(self, dt, z_error):

        # Calculate z position PID output
        z_pid_output = self.pid_z.compute(z_error, dt)

        # Calculate z position error compensator output
        z_error = self.pose_sp.z - self.z_mv
        z_error_compensator_term = self.pid_compensator_z.compute(z_error, dt)

        # Calculate z position switch function output
        z_pos_switch_output = self.z_pos_beta * math.tanh(
            deadzone(z_error_compensator_term / self.eps, -0.001, 0.001))

        # Calculate feed-forward term from z position reference
        z_ref_ff_term = self.z_feed_forward_gain * self.z_feed_forward_filter.compute(self.pose_sp.z)

        # Update status message
        self.status_msg.z_comp = self.z_compensator_gain * z_error_compensator_term
        self.status_msg.z_switch = z_pos_switch_output
        self.status_msg.z_ff = z_ref_ff_term
        self.status_msg.z_pid = z_pid_output

        # Calculate z velocity reference
        vel_ref = \
            z_pid_output + \
            z_pos_switch_output + \
            self.z_compensator_gain * z_error_compensator_term #+ \
            #z_ref_ff_term

        return vel_ref

    def calculate_yaw_rate_ref(self, dt, yaw_error):
        yaw_pid_term = self.pid_yaw.compute(yaw_error, dt)
        yaw_ff_term = self.yaw_feed_forward_gain * self.yaw_feed_forward_filter.compute(self.euler_sp.z)
        yaw_compensator_term = self.pid_compensator_yaw.compute(yaw_error, dt)
        yaw_switch_term = self.yaw_beta * math.tan(yaw_compensator_term / self.eps)

        # Update status message
        self.status_msg.yaw_comp = self.yaw_compensator_gain * yaw_compensator_term
        self.status_msg.yaw_pid = yaw_pid_term
        self.status_msg.yaw_ff = yaw_ff_term
        self.status_msg.yaw_switch = yaw_switch_term

        yaw_rate_ref = \
            yaw_pid_term #+ \
            # yaw_ff_term + \
            # self.yaw_compensator_gain * yaw_compensator_term + \
            # yaw_switch_term

        return yaw_rate_ref

    def calculate_yaw_rotor_velocity(self, dt, yaw_rate_error):
        yaw_rate_pid_term = self.pid_yaw_rate.compute(yaw_rate_error, dt)
        yaw_rate_ff_term = self.yaw_rate_feed_forward_gain * self.yaw_rate_feed_forward_filter.compute(
            self.euler_rate_sp.z)
        yaw_rate_compensator_term = self.pid_compensator_yaw_rate.compute(yaw_rate_error, dt)
        yaw_rate_switch_term = self.yaw_rate_beta * math.tan(yaw_rate_compensator_term / self.eps)

        # Update status message
        self.status_msg.yaw_rate_comp = self.yaw_rate_compensator_gain * yaw_rate_compensator_term
        self.status_msg.yaw_rate_pid = yaw_rate_pid_term
        self.status_msg.yaw_rate_ff = yaw_rate_ff_term
        self.status_msg.yaw_rate_switch = yaw_rate_switch_term

        yaw_rotor_offset = \
            yaw_rate_pid_term #+ \
            # yaw_rate_ff_term + \
            # self.yaw_rate_compensator_gain * yaw_rate_compensator_term + \
            # yaw_rate_switch_term

        return yaw_rotor_offset

    def calculate_rotor_velocities(self, dt, vz_error):

        # Calculate z velocity PID output
        vz_pid_output = self.pid_vz.compute(vz_error, dt)

        # Calculate feed forward term from z velocity reference
        vz_ref_ff_term = self.vz_feed_forward_gain * self.vz_feed_forward_filter.compute(self.vel_sp.z)

        # Calculate z velocity error compensator output
        vz_error_compensator_term = self.pid_compensator_vz.compute(vz_error, dt)

        # Calculate velocity switch function output
        vz_switch_output = self.vz_beta * math.tanh(
            deadzone(vz_error_compensator_term / self.eps, -0.001, 0.001))

        # Update status message
        self.status_msg.vz_comp = self.vz_compensator_gain * vz_error_compensator_term
        self.status_msg.vz_switch = vz_switch_output
        self.status_msg.vz_ff = vz_ref_ff_term
        self.status_msg.vz_pid = vz_pid_output

        # Calculate rotor velocity
        rotor_velocity = \
            vz_pid_output + \
            vz_switch_output + \
            self.vz_compensator_gain * vz_error_compensator_term + \
            vz_ref_ff_term

        return rotor_velocity


if __name__ == "__main__":

    rospy.init_node('smc_height_control', anonymous=True)
    try:
        height_control = SmcHeightController()
        height_control.run()
    except rospy.ROSInterruptException:
        pass


#!/usr/bin/env python
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

        self.cfg_server = Server(SmcPosCtlParamsConfig, self.cfg_callback)
        self.rate = 100.0
        self.Ts = 1.0 / self.rate 
        self.ros_rate = rospy.Rate(self.rate)
        self.t_start = rospy.Time.now()

        self.update_filter_coefficients()

        rospy.loginfo("Initialized position controller.")


    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag:
            print 'Waiting for velocity measurements.'
            rospy.sleep(0.5)
        print "Starting height control."

        self.t_old = rospy.Time.now()
        #self.t_old = datetime.now()

        while not rospy.is_shutdown():
            
            self.ros_rate.sleep()

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            self.t_old = t

            self.x_ref = self.filter_ref_a *  self.x_ref + (1.0-self.filter_ref_a) * self.x_sp
            vx_ref = self.pid_x.compute(self.x_ref - self.x_mv, self.Ts)
            pitch_r = self.pid_vx.compute(vx_ref - self.vx_mv, self.Ts)
            
            self.y_ref = self.filter_ref_a * self.y_ref + (1.0-self.filter_ref_a) *  self.y_sp
            vy_ref = self.pid_y.compute(self.y_ref - self.y_mv, self.Ts)
            roll_r = -self.pid_vy.compute(vy_ref - self.vy_mv, self.Ts)

            roll_ref = math.cos(self.euler_mv.z) * roll_r + math.sin(self.euler_mv.z) * pitch_r
            pitch_ref = -math.sin(self.euler_mv.z) * roll_r + math.cos(self.euler_mv.z) * pitch_r

            vec3_msg = Vector3(roll_ref, pitch_ref, 0)
            self.euler_ref_pub.publish(vec3_msg)

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

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('smc_pos_ctl')
    rospy.loginfo('Initializing position control')
    pos_ctl = SmcPositionControl()
    rospy.loginfo('Running position control')
    pos_ctl.run()


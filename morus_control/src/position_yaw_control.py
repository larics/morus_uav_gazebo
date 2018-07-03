#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import Float32, Int8
from dynamic_reconfigure.server import  Server
from dynamic_reconfigure.client import Client
from morus_msgs.cfg import MavPosCtlParamsConfig
from morus_msgs.msg import PIDController
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
import math
from datetime import datetime

class PositionControl:
    '''
    Class implements ROS node for cascade PID control of MAV position and yaw angle.
    Subscribes to:
        /morus/pose       - used to extract z-position of the vehicle
        /morus/velocity   - used to extract vz of the vehicle
        /morus/pos_ref    - used to set the reference for z-position
        /morus/vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        /morus/mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
        /morus/pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.x_sp = 0                   # x-position set point
        self.y_sp = 0                   # y-position set point
        self.z_sp = 0                   # z-position set point
        self.x_mv = 0                   # x-position measured value
        self.y_mv = 0                   # y-position measured value
        self.z_mv = 0                   # z-position measured value
        self.pid_x = PID()              # pid instance for x control
        self.pid_y = PID()              # pid instance for y control
        self.pid_z = PID()              # pid instance for z control


        self.vx_sp = 0                  # vx velocity set_point
        self.vy_sp = 0                  # vx velocity set_point
        self.vz_sp = 0                  # vz velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.vy_mv = 0                  # vy velocity measured value
        self.vz_mv = 0                  # vz velocity measured value
        self.vx_mv_old = 0              # vz velocity old measured value
        self.vy_mv_old = 0              # vz velocity old measured value
        self.vz_mv_old = 0              # vz velocity old measured value
        self.pid_vx = PID()             # pid instance for x-velocity control
        self.pid_vy = PID()             # pid instance for y-velocity control
        self.pid_vz = PID()             # pid instance for z-velocity control

        self.pid_yaw = PID()                    # yaw pid
        self.pid_yaw_rate = PID()               # yaw rate pid

        self.euler_mv = Vector3(0, 0, 0)        # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)        # euler angles referent values
        self.euler_rate_mv = Vector3(0, 0, 0)   # measured angular velocities
        self.dwz = 0
        self.yaw_ref = 0


        #########################################################
        #########################################################
        # Add parameters for z controller
        self.pid_z.set_kp(1.5)
        self.pid_z.set_ki(0.1)
        self.pid_z.set_kd(0.1)
        self.pid_z.set_lim_high(5)      # max vertical ascent speed
        self.pid_z.set_lim_low(-5)      # max vertical descent speed

        # Add parameters for vz controller
        self.pid_vz.set_kp(40)
        self.pid_vz.set_ki(0.1)
        self.pid_vz.set_kd(0.0)
        self.pid_vz.set_lim_high(350)   # max velocity of a motor
        self.pid_vz.set_lim_low(-350)   # min velocity of a motor

        self.pid_vx.set_kp(0.1)
        self.pid_vx.set_ki(0.0)
        self.pid_vx.set_kd(0)
        self.pid_vx.set_lim_high(0.2)
        self.pid_vx.set_lim_low(-0.2)

        self.pid_vy.set_kp(0.1)
        self.pid_vy.set_ki(0.0)
        self.pid_vy.set_kd(0)
        self.pid_vy.set_lim_high(0.2)
        self.pid_vy.set_lim_low(-0.2)

        self.pid_x.set_kp(0.5)
        self.pid_x.set_ki(0.0)
        self.pid_x.set_kd(0)
        self.pid_x.set_lim_high(5)
        self.pid_x.set_lim_low(-5)

        self.pid_y.set_kp(0.5)
        self.pid_y.set_ki(0.0)
        self.pid_y.set_kd(0)
        self.pid_y.set_lim_high(5)
        self.pid_y.set_lim_low(-5)

        self.pid_yaw.set_kp(1)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0)

        self.pid_yaw_rate.set_kp(100)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)
        
        #########################################################
        #########################################################
      
        self.mot_speed = 0              # referent motors velocity, computed by PID cascade

        self.t_old = 0
        self.x_ref = 0
        self.y_ref = 0
        self.z_ref = 0
        self.x_sp = 0
        self.y_sp = 0
        self.z_sp = 2
        self.yaw_sp = 0
        self.pos_ctl = Vector3(1,1,1)

        self.roll_vpc_command = 0 
        self.pitch_vpc_command = 0
        self.yaw_command = 0

        self.filter_const_meas = 0.9
        self.filter_const_ref = 5.0

        # cascade pid control
        self.thrust_const = 0.000456874
        self.vehicle_mass = 34 #30+30 +4
        self.mot_speed_hover = math.sqrt(self.vehicle_mass * 9.81 / 4 /  self.thrust_const) #432#432#527 # roughly

        
        rospy.Subscriber('position', PointStamped, self.pos_cb)
        rospy.Subscriber('velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('yaw_ref', Float32, self.yaw_ref_cb)
        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('attitude_rotor_command', Vector3, self.rotor_command_cb)


        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('pid_vz', PIDController, queue_size=1)
        self.pub_pid_x = rospy.Publisher('pid_x', PIDController, queue_size=1)
        self.pub_pid_vx = rospy.Publisher('pid_vx', PIDController, queue_size=1)
        self.pub_pid_y = rospy.Publisher('pid_y', PIDController, queue_size=1)
        self.pub_pid_vy = rospy.Publisher('pid_vy', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        
        self.euler_ref_pub = rospy.Publisher('euler_ref', Vector3, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
       
        self.cfg_server = Server(MavPosCtlParamsConfig, self.cfg_callback)
        
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

            # pt1 filter on reference
            self.z_ref = self.filter_ref_a * self.z_ref  + (1.0-self.filter_ref_a) * self.z_sp
            vz_ref = self.pid_z.compute(self.z_ref, self.z_mv, self.Ts)

            self.mot_speed = self.mot_speed_hover + \
                        self.pid_vz.compute(vz_ref, self.vz_mv, self.Ts)
            
            self.x_ref = self.filter_ref_a *  self.x_ref + (1.0-self.filter_ref_a) * self.x_sp
            vx_ref = self.pid_x.compute(self.x_ref, self.x_mv, self.Ts)
            pitch_r = self.pid_vx.compute(vx_ref, self.vx_mv, self.Ts)
            
            self.y_ref = self.filter_ref_a * self.y_ref + (1.0-self.filter_ref_a) *  self.y_sp
            vy_ref = self.pid_y.compute(self.y_ref, self.y_mv, self.Ts)
            roll_r = -self.pid_vy.compute(vy_ref, self.vy_mv, self.Ts)

            roll_ref = math.cos(self.euler_mv.z) * roll_r + math.sin(self.euler_mv.z) * pitch_r;
            pitch_ref = -math.sin(self.euler_mv.z) * roll_r + math.cos(self.euler_mv.z) * pitch_r;

            # yaw control
            yaw_rate_sv = self.pid_yaw.compute(self.yaw_sp, self.euler_mv.z, self.Ts)
            # yaw rate pid compute
            self.yaw_command = self.pid_yaw_rate.compute(yaw_rate_sv, self.euler_rate_mv.z, self.Ts)
            

            mot_speed_msg = Actuators()
            mot_speed1 = self.mot_speed + self.yaw_command 
            mot_speed2 = self.mot_speed - self.yaw_command 
            mot_speed3 = self.mot_speed + self.yaw_command 
            mot_speed4 = self.mot_speed - self.yaw_command 
            mot_speed_msg.angular_velocities = [mot_speed1, mot_speed2, mot_speed3, mot_speed4]
            self.pub_mot.publish(mot_speed_msg)
            
            vec3_msg = Vector3(roll_ref, pitch_ref, 0)
            self.euler_ref_pub.publish(vec3_msg)

            # Publish PID data - could be useful for tuning
            self.pub_pid_z.publish(self.pid_z.create_msg())
            self.pub_pid_vz.publish(self.pid_vz.create_msg())
            self.pub_pid_x.publish(self.pid_x.create_msg())
            self.pub_pid_vx.publish(self.pid_vx.create_msg())
            self.pub_pid_y.publish(self.pid_y.create_msg())
            self.pub_pid_vy.publish(self.pid_vy.create_msg())
            self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
            self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())

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
        self.vz_mv = self.filter_const_meas * self.vz_mv_old + (1 - self.filter_const_meas) * msg.twist.linear.z 
        self.vz_mv_old = self.vz_mv

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vx_sp = msg.x
        self.vy_sp = msg.y
        self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.x_sp = msg.x
        self.y_sp = msg.y
        self.z_sp = msg.z

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
            config.z_kp = self.pid_z.get_kp()
            config.z_ki = self.pid_z.get_ki()
            config.z_kd = self.pid_z.get_kd()

            config.vz_kp = self.pid_vz.get_kp()
            config.vz_ki = self.pid_vz.get_ki()
            config.vz_kd = self.pid_vz.get_kd()

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
            self.pid_z.set_kp(config.z_kp)
            self.pid_z.set_ki(config.z_ki)
            self.pid_z.set_kd(config.z_kd)

            self.pid_vz.set_kp(config.vz_kp)
            self.pid_vz.set_ki(config.vz_ki)
            self.pid_vz.set_kd(config.vz_kd)

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

    rospy.init_node('mav_pos_controller')
    rospy.loginfo('Initializing position control')
    pos_ctl = PositionControl()
    rospy.loginfo('Running position control')
    pos_ctl.run()


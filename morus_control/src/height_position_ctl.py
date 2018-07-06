#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from std_msgs.msg import Float32
from dynamic_reconfigure.server import  Server
from morus_msgs.cfg import MavZCtlParamsConfig
from morus_msgs.msg import PIDController
from sensor_msgs.msg import Imu
from mav_msgs.msg import Actuators
import math

class HeightControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        pose       - used to extract z-position of the vehicle
        velocity   - used to extract vz of the vehicle
        pos_ref    - used to set the reference for z-position
        vel_ref    - used to set the reference for vz-position (useful for testing velocity controller)

    Publishes:
        mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
        pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component (useful for tuning params)
        pid_vz        - publishes PID-vz data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.z_sp = 0                   # z-position set point
        self.z_ref_filt = 0             # z ref filtered
        self.z_mv = 0                   # z-position measured value
        self.pid_z = PID()              # pid instance for z control

        self.vz_sp = 0                  # vz velocity set_point
        self.vz_mv = 0                   # vz velocity measured value
        self.pid_vz = PID()             # pid instance for z-velocity control

        self.pid_yaw_rate = PID()
        self.euler_mv = Vector3(0, 0, 0)           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles referent values
        self.euler_rate_mv = Vector3(0, 0, 0)      # measured angular velocities
        self.dwz = 0

        #########################################################
        #########################################################
        # Add parameters for z controller
        self.pid_z.set_kp(0.5)
        self.pid_z.set_ki(0.01)
        self.pid_z.set_kd(0.75)

        # Add parameters for vz controller
        self.pid_vz.set_kp(20)#87.2)
        self.pid_vz.set_ki(0.1)
        self.pid_vz.set_kd(10)#10.89)

        # Yaw rate params
        self.pid_yaw_rate.set_kp(75)
        self.pid_yaw_rate.set_ki(5)
        self.pid_yaw_rate.set_kd(15)
        #########################################################
        #########################################################


        self.pid_z.set_lim_high(5)      # max vertical ascent speed
        self.pid_z.set_lim_low(-5)      # max vertical descent speed

        self.pid_vz.set_lim_high(350)   # max velocity of a motor
        self.pid_vz.set_lim_low(-350)   # min velocity of a motor

        self.mot_speed = 0              # referent motors velocity, computed by PID cascade

        self.gm_attitude_ctl = 0        # flag indicates if attitude control is turned on
        self.gm_attitude_ctl = rospy.get_param('~gm_attitude_ctl')

        self.t_old = 0

        rospy.Subscriber('pose_with_covariance', PoseWithCovarianceStamped, self.pose_cb)
        rospy.Subscriber('velocity', TwistStamped, self.vel_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('imu', Imu, self.ahrs_cb)

        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('pid_vz', PIDController, queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.cfg_server = Server(MavZCtlParamsConfig, self.cfg_callback)
        self.ros_rate = rospy.Rate(10)
        self.t_start = rospy.Time.now()

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''
        self.t_old = rospy.Time.now()
        #self.t_old = datetime.now()

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            ########################################################
            ########################################################
            # Implement cascade PID control here.
            # Reference for z is stored in self.z_sp.
            # Measured z-position is stored in self.z_mv.
            # If you want to test only vz - controller, the corresponding reference is stored in self.vz_sp.
            # Measured vz-velocity is stored in self.vz_mv
            # Resultant referent value for motor velocity should be stored in variable mot_speed.
            # When you implement attitude control set the flag self.attitude_ctl to 1

            #self.gm_attitude_ctl = 1  # don't forget to set me to 1 when you implement attitude ctl
	    if not self.start_flag:
	      print 'Waiting for velocity measurements.'
	      rospy.sleep(0.5)
	    else:
	      t = rospy.Time.now()
	      dt = (t - self.t_old).to_sec()
	      #t = datetime.now()
	      #dt = (t - self.t_old).total_seconds()
	      if dt > 0.105 or dt < 0.095:
		  print dt

	      self.t_old = t
	      #Corrections for HIL
	      self.mot_speed_hover = 427
	      #self.mot_speed_hover = 280
	      # prefilter for reference
	      a = 0.1
	      self.z_ref_filt = (1-a) * self.z_ref_filt  + a * self.z_sp
	      vz_ref = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
	      self.mot_speed = self.mot_speed_hover + \
			  self.pid_vz.compute(vz_ref, self.vz_mv, dt)

	      self.dwz = self.pid_yaw_rate.compute(self.euler_sp.z, self.euler_rate_mv.z, dt)
	      ########################################################
	      ########################################################


	      if self.gm_attitude_ctl == 0:
		  # moving masses are used to control attitude
		  # Publish motor velocities
		  mot_speed_msg = Actuators()
		  mot_speed1 = self.mot_speed + self.dwz
		  mot_speed2 = self.mot_speed - self.dwz
		  mot_speed3 = self.mot_speed + self.dwz
		  mot_speed4 = self.mot_speed - self.dwz
		  mot_speed_msg.angular_velocities = [mot_speed1, mot_speed2, mot_speed3, mot_speed4]
		  self.pub_mot.publish(mot_speed_msg)
	      else:
		  # gas motors are used to control attitude
		  # publish referent motor velocity to attitude controller
		  mot_speed_msg = Float32(self.mot_speed)
		  self.mot_ref_pub.publish(mot_speed_msg)


	      # Publish PID data - could be useful for tuning
	      self.pub_pid_z.publish(self.pid_z.create_msg())
	      self.pub_pid_vz.publish(self.pid_vz.create_msg())

    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        self.z_mv = msg.pose.pose.position.z

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True
        self.vz_mv = msg.twist.linear.z

    def vel_ref_cb(self, msg):
        '''
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3
        '''
        self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        '''
        Referent position callback. Received value (z-component) is used as a referent height.
        :param msg: Type Vector3
        '''
        self.z_sp = msg.z

    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
        '''
        if not self.start_flag:
            self.start_flag = True

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

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.z_kp = self.pid_z.get_kp()
            config.z_ki = self.pid_z.get_ki()
            config.z_kd = self.pid_z.get_kd()

            config.vz_kp = self.pid_vz.get_kp()
            config.vz_ki = self.pid_vz.get_ki()
            config.vz_kd = self.pid_vz.get_kd()

            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_z.set_kp(config.z_kp)
            self.pid_z.set_ki(config.z_ki)
            self.pid_z.set_kd(config.z_kd)

            self.pid_vz.set_kp(config.vz_kp)
            self.pid_vz.set_ki(config.vz_ki)
            self.pid_vz.set_kd(config.vz_kd)

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_z_controller')
    height_ctl = HeightControl()
    height_ctl.run()


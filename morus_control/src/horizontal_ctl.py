#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from dynamic_reconfigure.server import Server
from morus_msgs.cfg import MavXYCtlParamsConfig
from morus_msgs.msg import PIDController
from morus_msgs.msg import AngleControlFlapRef
import math
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from datetime import datetime

class HorizontalControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
    Subscribes to:
        /morus/pose       - used to extract x any y position of the vehicle
        /morus/imu        - used to extract measured yaw
        /morus/velocity   - used to extract vx and vy of the vehicle
        /morus/pos_ref    - used to set the reference for x and y -position
        /morus/vel_ref    - used to set the reference for vx and vy-position (useful for testing velocity controller)

    Publishes:
        /morus/euler_ref  - publishes referent value for euler angles (roll, pitch, yaw)
        /morus/pid_x      - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_vx     - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_y      - publishes PID-x data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_vy     - publishes PID-vx data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controller params online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''
        self.clock = Clock()

        self.start_flag = False         # indicates if we received the first measurement
        self.config_start = False       # flag indicates if the config callback is called for the first time

        self.roll_sp = 0                #referent roll value
        self.pitch_sp = 0               #referent pitch value

        self.yaw_sp = 0                 # referent yaw value
        self.yaw_mv = 0                 # measured yaw value

        self.x_sp = 0                   # x-position set point
        self.x_mv = 0                   # x-position measured value
        self.pid_x = PID()              # pid instance for x control

        self.vx_sp = 0                  # vx velocity set_point
        self.vx_mv = 0                  # vx velocity measured value
        self.pid_vx = PID()             # pid instance for x-velocity control

        self.y_sp = 0                   # y-position set point
        self.y_mv = 0                   # y-position measured value
        self.pid_y = PID()              # pid instance for y control

        self.vy_sp = 0                  # vy velocity set_point
        self.vy_mv = 0                  # vy velocity measured value
        self.pid_vy = PID()             # pid instance for y-velocity control

        #########################################################
        #########################################################
        # Add parameters for x controller
        self.pid_x.set_kp(0.5)
        self.pid_x.set_ki(0.001)
        self.pid_x.set_kd(0.001)

        # Add parameters for vx controller
        self.pid_vx.set_kp(1)
        self.pid_vx.set_ki(0.008)
        self.pid_vx.set_kd(0.02)

        # Add parameters for y controller
        self.pid_y.set_kp(0.5)
        self.pid_y.set_ki(0.001)
        self.pid_y.set_kd(0.001)

        # Add parameters for vy controller
        self.pid_vy.set_kp(1)
        self.pid_vy.set_ki(0.008)
        self.pid_vy.set_kd(0.02)
        #########################################################
        #########################################################

        #self.pid_x.set_lim_high(15)        # max vx speed
        #self.pid_x.set_lim_low(-15)      # min vx speed

        #self.pid_vx.set_lim_high(15/180 * math.pi)        # max wing angle
        #self.pid_vx.set_lim_low(-15/180 * math.pi)      # min wing angle

        #self.pid_y.set_lim_high(15)        # max vy speed
        #self.pid_y.set_lim_low(-15)      # min vy speed

        #self.pid_vy.set_lim_high(15/180 * math.pi)        # max wing angle
        #self.pid_vy.set_lim_low(-15.0/180 * math.pi)      # min wing angle

        rospy.Subscriber('pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('odometry', Odometry, self.vel_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        rospy.Subscriber('yaw_ref', Float32, self.yaw_ref_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)

        self.angle_control_flap_command_pub_0 = rospy.Publisher('angle_wing_0_ref_value', AngleControlFlapRef, queue_size=1)
        self.angle_control_flap_command_pub_1 = rospy.Publisher('angle_wing_1_ref_value', AngleControlFlapRef, queue_size=1)
        self.angle_control_flap_command_pub_2 = rospy.Publisher('angle_wing_2_ref_value', AngleControlFlapRef, queue_size=1)
        self.angle_control_flap_command_pub_3 = rospy.Publisher('angle_wing_3_ref_value', AngleControlFlapRef, queue_size=1)

        self.pub_pid_x = rospy.Publisher('pid_x', PIDController, queue_size=1)
        self.pub_pid_vx = rospy.Publisher('pid_vx', PIDController, queue_size=1)
        self.pub_pid_y = rospy.Publisher('pid_y', PIDController, queue_size=1)
        self.pub_pid_vy = rospy.Publisher('pid_vy', PIDController, queue_size=1)
        self.euler_ref_pub = rospy.Publisher('euler_ref', Vector3, queue_size=1)
        self.cfg_server = Server(MavXYCtlParamsConfig, self.cfg_callback)
        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(float(self.rate))

        #init_msg = Float64()
        #init_msg.data = 0

        #self.angle_control_flap_command_pub_0.publish(init_msg)
        #self.angle_control_flap_command_pub_1.publish(init_msg)
        #self.angle_control_flap_command_pub_2.publish(init_msg)
        #self.angle_control_flap_command_pub_3.publish(init_msg)
        #angle_wing_ref_value TODO
        #self.init_pub_0.publish(init_msg)
        #self.init_pub_1.publish(init_msg)
        #self.init_pub_2.publish(init_msg)
        #self.init_pub_3.publish(init_msg)


    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''
        while (rospy.get_time() == 0) and (not rospy.is_shutdown()):
            print 'Waiting for clock server to start'
        print 'Received first clock message'

        while (not self.start_flag) and (not rospy.is_shutdown()):
            print "Waiting for the first measurement."
            rospy.sleep(0.5)
        print "Starting horizontal control."

        self.t_old = rospy.Time.now()
        clock_old = self.clock
        #self.t_old = datetime.now()
        self.count = 0

        while not rospy.is_shutdown():
            #self.ros_rate.sleep()
            while (not self.start_flag) and (not rospy.is_shutdown()):
                print "Waiting for the first measurement."
                rospy.sleep(0.5)
            rospy.sleep(1.0/float(self.rate))

            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()

            clock_old = clock_now
            if dt_clk > (1.0 / self.rate + 0.005):
                self.count += 1
                #print self.count, ' - ',  dt_clk

            if dt_clk < (1.0 / self.rate - 0.005):
                self.count += 1
                #print self.count, ' - ',  dt_clk

            if dt_clk < 0.005:
                dt_clk = 0.01


            ########################################################
            ########################################################
            # Implement cascade PID control here.
            # Reference for x is stored in self.x_sp.
            # Measured x-position is stored in self.x_mv.
            # If you want to test only vx - controller, the corresponding reference is stored in self.vx_sp.
            # Measured vx-velocity is stored in self.vx_mv
            # Reference for y is stored in self.y_sp.
            # Measured y-position is stored in self.y_mv.
            # If you want to test only vy - controller, the corresponding reference is stored in self.vy_sp.
            # Measured vx-velocity is stored in self.vy_mv
            # Measured yaw angle is stored in self.yaw_mv (in rad)
            # Resultant referent value for roll and pitch (in mobile coordinate system!)
            # should be stored in variable roll_sp and pitch_sp
            
            #x component
            #print "dt_clk", dt_clk
            #print "self.x_sp", self.x_sp
            #print "self.x_mv", self.x_mv
            self.vx_sp =self.pid_x.compute(self.x_sp,self.x_mv,dt_clk)
            #print "vx_sp", self.vx_sp
            #print "self.vx_mv", self.vx_mv
            angle_control_flap_x =self.pid_vx.compute(self.vx_sp,self.vx_mv,dt_clk)
            #print "angle_control_flap_x", angle_control_flap_x
            
            #y component
            #print "self.y_sp", self.y_sp
            #print "self.y_mv", self.y_mv
            self.vy_sp=self.pid_y.compute(self.y_sp,self.y_mv,dt_clk)
            #print "vy_sp", self.vy_sp
            #print "self.vy_mv", self.vy_mv
            angle_control_flap_y =self.pid_vy.compute(self.vy_sp,self.vy_mv,dt_clk)
            #print "angle_control_flap_y", angle_control_flap_y
            #print "\n"
            angle_control_flap_x_tf = math.cos(self.yaw_mv) * angle_control_flap_x + math.sin(self.yaw_mv) * angle_control_flap_y;
            angle_control_flap_y_tf = -math.sin(self.yaw_mv) * angle_control_flap_x + math.cos(self.yaw_mv) * angle_control_flap_y;

            ########################################################
            ########################################################
            angle_control_flap_command_pub_0_msg = AngleControlFlapRef()
            angle_control_flap_command_pub_0_msg.anglecontrolflap = angle_control_flap_x_tf
            angle_control_flap_command_pub_1_msg = AngleControlFlapRef()
            angle_control_flap_command_pub_1_msg.anglecontrolflap = angle_control_flap_y_tf
            angle_control_flap_command_pub_2_msg = AngleControlFlapRef()
            angle_control_flap_command_pub_2_msg.anglecontrolflap = angle_control_flap_x_tf
            angle_control_flap_command_pub_3_msg = AngleControlFlapRef()
            angle_control_flap_command_pub_3_msg.anglecontrolflap = angle_control_flap_y_tf

            self.angle_control_flap_command_pub_0.publish(angle_control_flap_command_pub_0_msg)
            self.angle_control_flap_command_pub_1.publish(angle_control_flap_command_pub_1_msg)
            self.angle_control_flap_command_pub_2.publish(angle_control_flap_command_pub_2_msg)
            self.angle_control_flap_command_pub_3.publish(angle_control_flap_command_pub_3_msg)

            #euler_sv = Vector3(self.roll_sp, self.pitch_sp, self.yaw_sp)
            #self.euler_ref_pub.publish(euler_sv)

            # Publish PID data - could be usefule for tuning
            self.pub_pid_x.publish(self.pid_x.create_msg())
            self.pub_pid_vx.publish(self.pid_vx.create_msg())
            self.pub_pid_y.publish(self.pid_y.create_msg())
            self.pub_pid_vy.publish(self.pid_vy.create_msg())


    def pose_cb(self, msg):
        '''
        Pose (6DOF - position and orientation) callback.
        :param msg: Type PoseWithCovarianceStamped
        '''
        if not self.start_flag:
            self.start_flag = True
        self.x_mv = msg.pose.position.x
        self.y_mv = msg.pose.position.y

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
        self.roll_mv = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.pitch_mv = math.asin(2 * (qw * qy - qx * qz))
        self.yaw_mv = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        if not self.start_flag:
            self.start_flag = True
        self.vx_mv = msg.twist.twist.linear.x
        self.vy_mv = msg.twist.twist.linear.y

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

    def yaw_ref_cb(self, msg):
        '''
        Referent yaw callback. Received value is used as a referent yaw (heading).
        :param msg: Type Float32
        '''
        self.yaw_sp = msg.data

    def clock_cb(self, msg):
        self.clock = msg

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
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


        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('mav_xy_controller')
    xy_ctl = HorizontalControl()
    xy_ctl.run()

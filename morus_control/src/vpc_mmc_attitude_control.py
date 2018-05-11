#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, Empty
from morus_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
from morus_control.cfg import VpcMmcuavAttitudeCtlParamsConfig
import math
from datetime import datetime
from rosgraph_msgs.msg import Clock
import simple_filters
import copy

class AttitudeControl:
    '''
    Class implements MAV attitude control (roll, pitch, yaw). Two PIDs in cascade are
    used for each degree of freedom.
    Subscribes to:
        /morus/imu                - used to extract attitude and attitude rate of the vehicle
        /morus/mot_vel_ref        - used to receive referent motor velocity from the height controller
        /morus/euler_ref          - used to set the attitude referent (useful for testing controllers)
    Publishes:
        /morus/command/motors     - referent motor velocities sent to each motor controller
        /morus/pid_roll           - publishes PID-roll data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_roll_rate      - publishes PID-roll_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch          - publishes PID-pitch data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_pitch_rate     - publishes PID-pitch_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw            - publishes PID-yaw data - referent value, measured value, P, I, D and total component (useful for tuning params)
        /morus/pid_yaw_rate       - publishes PID-yaw_rate data - referent value, measured value, P, I, D and total component (useful for tuning params)

    Dynamic reconfigure is used to set controllers param online.
    '''

    def __init__(self):
        '''
        Initialization of the class.
        '''

        self.start_flag = False             # flag indicates if the first measurement is received
        self.config_start = False           # flag indicates if the config callback is called for the first time
        self.euler_mv = Vector3()           # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)    # euler angles referent values
        self.euler_sp_old = Vector3(0, 0, 0)
        self.euler_sp_filt = Vector3(0, 0, 0)

        self.w_sp = 0                       # referent value for motor velocity - it should be the output of height controller

        self.euler_rate_mv = Vector3()      # measured angular velocities
        self.euler_rate_mv_old = Vector3()

        self.clock = Clock()

        self.pid_roll = PID()                           # roll controller
        self.pid_roll_rate  = PID()                     # roll rate (wx) controller

        self.pid_pitch = PID()                          # pitch controller
        self.pid_pitch_rate = PID()                     # pitch rate (wy) controller

        self.pid_yaw = PID()                            # yaw controller
        self.pid_yaw_rate = PID()                       # yaw rate (wz) controller

        # Adding VPC controllers for roll and pitch
        self.pid_vpc_roll = PID()
        self.pid_vpc_pitch = PID()

        ##################################################################
        ##################################################################
        # Add your PID params here

        self.pid_roll.set_kp(3.0)
        self.pid_roll.set_ki(1.0)
        self.pid_roll.set_kd(0.0)

        self.pid_roll_rate.set_kp(2.5)
        self.pid_roll_rate.set_ki(0.0)
        self.pid_roll_rate.set_kd(0.0)
        self.pid_roll_rate.set_lim_high(0.3)
        self.pid_roll_rate.set_lim_low(-0.3)

        self.pid_pitch.set_kp(3.0)
        self.pid_pitch.set_ki(1.0)
        self.pid_pitch.set_kd(0.0)

        self.pid_pitch_rate.set_kp(2.5)
        self.pid_pitch_rate.set_ki(0.0)
        self.pid_pitch_rate.set_kd(0.0)
        self.pid_pitch_rate.set_lim_high(0.3)
        self.pid_pitch_rate.set_lim_low(-0.3)

        self.pid_yaw.set_kp(1.0)
        self.pid_yaw.set_ki(0)
        self.pid_yaw.set_kd(0.1)

        self.pid_yaw_rate.set_kp(75.0)
        self.pid_yaw_rate.set_ki(0)
        self.pid_yaw_rate.set_kd(0)

        # VPC pids
        self.pid_vpc_roll.set_kp(0)
        self.pid_vpc_roll.set_ki(0.0)
        self.pid_vpc_roll.set_kd(0)
        self.pid_vpc_roll.set_lim_high(400)
        self.pid_vpc_roll.set_lim_low(-400)

        self.pid_vpc_pitch.set_kp(0)
        self.pid_vpc_pitch.set_ki(0.0)
        self.pid_vpc_pitch.set_kd(0)
        self.pid_vpc_pitch.set_lim_high(400)
        self.pid_vpc_pitch.set_lim_low(-400)

        # Filter parameters
        self.rate_mv_filt_K = 1.0
        self.rate_mv_filt_T = 0.02
        # Reference prefilters
        self.roll_reference_prefilter_K = 1.0
        self.roll_reference_prefilter_T = 0.0
        self.pitch_reference_prefilter_K = 1.0
        self.pitch_reference_prefilter_T = 0.0

        # Offsets for pid outputs
        self.roll_rate_output_trim = 0.0
        self.pitch_rate_output_trim = 0.0


        ##################################################################
        ##################################################################

        self.rate = rospy.get_param('~rate', 100)
        self.ros_rate = rospy.Rate(self.rate)                 # attitude control at 100 Hz
        self.Ts = 1.0/float(self.rate)

        self.t_old = 0

        rospy.Subscriber('imu', Imu, self.ahrs_cb)
        rospy.Subscriber('mot_vel_ref', Float64, self.mot_vel_ref_cb)
        rospy.Subscriber('euler_ref', Vector3, self.euler_ref_cb)
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber('reset_controllers', Empty, self.reset_controllers_cb)

        #self.pub_mass0 = rospy.Publisher('movable_mass_0_position_controller/command', Float64, queue_size=1)
        #self.pub_mass1 = rospy.Publisher('movable_mass_1_position_controller/command', Float64, queue_size=1)
        #self.pub_mass2 = rospy.Publisher('movable_mass_2_position_controller/command', Float64, queue_size=1)
        #self.pub_mass3 = rospy.Publisher('movable_mass_3_position_controller/command', Float64, queue_size=1)
        self.attitude_pub = rospy.Publisher('attitude_command', Float64MultiArray, queue_size=1)
        self.pub_pid_roll = rospy.Publisher('pid_roll', PIDController, queue_size=1)
        self.pub_pid_roll_rate = rospy.Publisher('pid_roll_rate', PIDController, queue_size=1)
        self.pub_pid_pitch = rospy.Publisher('pid_pitch', PIDController, queue_size=1)
        self.pub_pid_pitch_rate = rospy.Publisher('pid_pitch_rate', PIDController, queue_size=1)
        self.pub_pid_yaw = rospy.Publisher('pid_yaw', PIDController, queue_size=1)
        self.pub_pid_yaw_rate = rospy.Publisher('pid_yaw_rate', PIDController, queue_size=1)
        self.pub_pid_vpc_roll = rospy.Publisher('pid_vpc_roll', PIDController, queue_size=1)
        self.pub_pid_vpc_pitch = rospy.Publisher('pid_vpc_pitch', PIDController, queue_size=1)
        self.cfg_server = Server(VpcMmcuavAttitudeCtlParamsConfig, self.cfg_callback)

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for cascade attitude control.
        '''

        while (rospy.get_time() == 0) and (not rospy.is_shutdown()):
            print 'Waiting for clock server to start'

        print 'Received first clock message'

        while (not self.start_flag) and (not rospy.is_shutdown()):
            print "Waiting for the first measurement."
            rospy.sleep(0.5)
        print "Starting attitude control."

        self.t_old = rospy.Time.now()
        clock_old = self.clock
        #self.t_old = datetime.now()
        self.count = 0
        self.loop_count = 0

        while not rospy.is_shutdown():
            #self.ros_rate.sleep()
            while (not self.start_flag) and (not rospy.is_shutdown()):
                print "Waiting for the first measurement."
                rospy.sleep(0.5)
            rospy.sleep(1.0/float(self.rate))

            self.euler_sp_filt.x = simple_filters.filterPT1(self.euler_sp_old.x, 
                self.euler_sp.x, self.roll_reference_prefilter_T, self.Ts, 
                self.roll_reference_prefilter_K)
            self.euler_sp_filt.y = simple_filters.filterPT1(self.euler_sp_old.y, 
                self.euler_sp.y, self.pitch_reference_prefilter_T, self.Ts, 
                self.pitch_reference_prefilter_K)
            self.euler_sp_filt.z = self.euler_sp.z
            #self.euler_sp.z = simple_filters.filterPT1(self.euler_sp_old.z, self.euler_sp.z, 0.2, self.Ts, 1.0)

            self.euler_sp_old = copy.deepcopy(self.euler_sp_filt)

            clock_now = self.clock
            dt_clk = (clock_now.clock - clock_old.clock).to_sec()

            clock_old = clock_now
            if dt_clk > (1.0 / self.rate + 0.005):
                self.count += 1
                print self.count, ' - ',  dt_clk

            if dt_clk < (1.0 / self.rate - 0.005):
                self.count += 1
                print self.count, ' - ',  dt_clk

            if dt_clk < 0.005:
                dt_clk = 0.01

            # Roll
            roll_rate_sv = self.pid_roll.compute(self.euler_sp_filt.x, self.euler_mv.x, dt_clk)
            # roll rate pid compute
            roll_rate_output = self.pid_roll_rate.compute(roll_rate_sv, self.euler_rate_mv.x, dt_clk) + self.roll_rate_output_trim

            # Pitch
            pitch_rate_sv = self.pid_pitch.compute(self.euler_sp_filt.y, self.euler_mv.y, dt_clk)
            # pitch rate pid compute
            pitch_rate_output = self.pid_pitch_rate.compute(pitch_rate_sv, self.euler_rate_mv.y, dt_clk) + self.pitch_rate_output_trim

            # Yaw
            yaw_rate_sv = self.pid_yaw.compute(self.euler_sp_filt.z, self.euler_mv.z, dt_clk)
            # yaw rate pid compute
            yaw_rate_output = self.pid_yaw_rate.compute(yaw_rate_sv, self.euler_rate_mv.z, dt_clk)

            # VPC stuff
            vpc_roll_output = -self.pid_vpc_roll.compute(0.0, roll_rate_output, dt_clk)
            # Due to some wiring errors we set output to +, should be -
            vpc_pitch_output = -self.pid_vpc_pitch.compute(0.0, pitch_rate_output, dt_clk)


            # Publish mass position
            #mass0_command_msg = Float64()
            #mass0_command_msg.data = dx_pitch
            #mass2_command_msg = Float64()
            #mass2_command_msg.data = -dx_pitch
            #mass1_command_msg = Float64()
            #mass1_command_msg.data = -dy_roll
            #mass3_command_msg = Float64()
            #mass3_command_msg.data = dy_roll
            #self.pub_mass0.publish(mass0_command_msg)
            #self.pub_mass1.publish(mass1_command_msg)
            #self.pub_mass2.publish(mass2_command_msg)
            #self.pub_mass3.publish(mass3_command_msg)

            # Publish attitude
            attitude_output = Float64MultiArray()
            attitude_output.data = [roll_rate_output, pitch_rate_output, \
                yaw_rate_output, vpc_roll_output, vpc_pitch_output]
            self.attitude_pub.publish(attitude_output)

            # Publish PID data - could be usefule for tuning
            self.pub_pid_roll.publish(self.pid_roll.create_msg())
            self.pub_pid_roll_rate.publish(self.pid_roll_rate.create_msg())
            self.pub_pid_pitch.publish(self.pid_pitch.create_msg())
            self.pub_pid_pitch_rate.publish(self.pid_pitch_rate.create_msg())
            self.pub_pid_yaw.publish(self.pid_yaw.create_msg())
            self.pub_pid_yaw_rate.publish(self.pid_yaw_rate.create_msg())
            # Publish VPC pid data
            self.pub_pid_vpc_roll.publish(self.pid_vpc_roll.create_msg())
            self.pub_pid_vpc_pitch.publish(self.pid_vpc_pitch.create_msg())

    def mot_vel_ref_cb(self, msg):
        '''
        Referent motor velocity callback. (This should be published by height controller).
        :param msg: Type Float32
        '''
        self.w_sp = msg.data

    def reset_controllers_cb(self, msg):
        self.start_flag = False
        self.pid_pitch.reset()
        self.pid_pitch_rate.reset()
        self.pid_roll.reset()
        self.pid_roll_rate.reset()
        self.pid_yaw.reset()
        self.pid_yaw_rate.reset()
        self.pid_vpc_pitch.reset()
        self.pid_vpc_roll.reset()
        rospy.Subscriber('imu', Imu, self.ahrs_cb)

    def ahrs_cb(self, msg):
        '''
        AHRS callback. Used to extract roll, pitch, yaw and their rates.
        We used the following order of rotation - 1)yaw, 2) pitch, 3) roll
        :param msg: Type sensor_msgs/Imu
        '''

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

        # If we are in first pass initialize filter
        if not self.start_flag:
            self.start_flag = True
            self.euler_rate_mv_old = copy.deepcopy(self.euler_rate_mv)

        # Filtering angular velocities
        self.euler_rate_mv.x = simple_filters.filterPT1(self.euler_rate_mv_old.x, 
            self.euler_rate_mv.x, self.rate_mv_filt_T, self.Ts, self.rate_mv_filt_K)
        self.euler_rate_mv.y = simple_filters.filterPT1(self.euler_rate_mv_old.y, 
            self.euler_rate_mv.y, self.rate_mv_filt_T, self.Ts, self.rate_mv_filt_K)
        self.euler_rate_mv.z = simple_filters.filterPT1(self.euler_rate_mv_old.z, 
            self.euler_rate_mv.z, self.rate_mv_filt_T, self.Ts, self.rate_mv_filt_K)

        # Set old to current
        self.euler_rate_mv_old = copy.deepcopy(self.euler_rate_mv)

    def euler_ref_cb(self, msg):
        '''
        Euler ref values callback.
        :param msg: Type Vector3 (x-roll, y-pitch, z-yaw)
        '''
        self.euler_sp = msg

    def clock_cb(self, msg):
        self.clock = msg

    def cfg_callback(self, config, level):
        """ Callback for dynamically reconfigurable parameters (P,I,D gains for each controller)
        """

        if not self.config_start:
            # callback is called for the first time. Use this to set the new params to the config server
            config.roll_kp = self.pid_roll.get_kp()
            config.roll_ki = self.pid_roll.get_ki()
            config.roll_kd = self.pid_roll.get_kd()

            config.roll_r_kp = self.pid_roll_rate.get_kp()
            config.roll_r_ki = self.pid_roll_rate.get_ki()
            config.roll_r_kd = self.pid_roll_rate.get_kd()

            config.pitch_kp = self.pid_pitch.get_kp()
            config.pitch_ki = self.pid_pitch.get_ki()
            config.pitch_kd = self.pid_pitch.get_kd()

            config.pitch_r_kp = self.pid_pitch_rate.get_kp()
            config.pitch_r_ki = self.pid_pitch_rate.get_ki()
            config.pitch_r_kd = self.pid_pitch_rate.get_kd()

            config.yaw_kp = self.pid_yaw.get_kp()
            config.yaw_ki = self.pid_yaw.get_ki()
            config.yaw_kd = self.pid_yaw.get_kd()

            config.yaw_r_kp = self.pid_yaw_rate.get_kp()
            config.yaw_r_ki = self.pid_yaw_rate.get_ki()
            config.yaw_r_kd = self.pid_yaw_rate.get_kd()

            # VPC pids
            config.vpc_roll_kp = self.pid_vpc_roll.get_kp()
            config.vpc_roll_ki = self.pid_vpc_roll.get_ki()
            config.vpc_roll_kd = self.pid_vpc_roll.get_kd()

            config.vpc_pitch_kp = self.pid_vpc_pitch.get_kp()
            config.vpc_pitch_ki = self.pid_vpc_pitch.get_ki()
            config.vpc_pitch_kd = self.pid_vpc_pitch.get_kd()

            # Rate filter
            config.rate_mv_filt_K = self.rate_mv_filt_K
            config.rate_mv_filt_T = self.rate_mv_filt_T
            # Roll and pitch reference prefilters
            config.roll_reference_prefilter_K = self.roll_reference_prefilter_K
            config.roll_reference_prefilter_T = self.roll_reference_prefilter_T
            config.pitch_reference_prefilter_K = self.pitch_reference_prefilter_K
            config.pitch_reference_prefilter_T = self.pitch_reference_prefilter_T

            # Rate output offsets
            config.roll_rate_output_trim = self.roll_rate_output_trim
            config.pitch_rate_output_trim = self.pitch_rate_output_trim


            self.config_start = True
        else:
            # The following code just sets up the P,I,D gains for all controllers
            self.pid_roll.set_kp(config.roll_kp)
            self.pid_roll.set_ki(config.roll_ki)
            self.pid_roll.set_kd(config.roll_kd)

            self.pid_roll_rate.set_kp(config.roll_r_kp)
            self.pid_roll_rate.set_ki(config.roll_r_ki)
            self.pid_roll_rate.set_kd(config.roll_r_kd)

            self.pid_pitch.set_kp(config.pitch_kp)
            self.pid_pitch.set_ki(config.pitch_ki)
            self.pid_pitch.set_kd(config.pitch_kd)

            self.pid_pitch_rate.set_kp(config.pitch_r_kp)
            self.pid_pitch_rate.set_ki(config.pitch_r_ki)
            self.pid_pitch_rate.set_kd(config.pitch_r_kd)

            self.pid_yaw.set_kp(config.yaw_kp)
            self.pid_yaw.set_ki(config.yaw_ki)
            self.pid_yaw.set_kd(config.yaw_kd)

            self.pid_yaw_rate.set_kp(config.yaw_r_kp)
            self.pid_yaw_rate.set_ki(config.yaw_r_ki)
            self.pid_yaw_rate.set_kd(config.yaw_r_kd)

            # VPC pids
            self.pid_vpc_roll.set_kp(config.vpc_roll_kp)
            self.pid_vpc_roll.set_ki(config.vpc_roll_ki)
            self.pid_vpc_roll.set_kd(config.vpc_roll_kd)

            self.pid_vpc_pitch.set_kp(config.vpc_pitch_kp)
            self.pid_vpc_pitch.set_ki(config.vpc_pitch_ki)
            self.pid_vpc_pitch.set_kd(config.vpc_pitch_kd)

            # Rate filter
            self.rate_mv_filt_K = config.rate_mv_filt_K
            self.rate_mv_filt_T = config.rate_mv_filt_T
            # Roll and pitch reference prefilters
            self.roll_reference_prefilter_K = config.roll_reference_prefilter_K
            self.roll_reference_prefilter_T = config.roll_reference_prefilter_T
            self.pitch_reference_prefilter_K = config.pitch_reference_prefilter_K
            self.pitch_reference_prefilter_T = config.pitch_reference_prefilter_T

            # Rate output offsets
            self.roll_rate_output_trim = config.roll_rate_output_trim
            self.pitch_rate_output_trim = config.pitch_rate_output_trim

        # this callback should return config data back to server
        return config

if __name__ == '__main__':

    rospy.init_node('vpc_mmcuav_attitude_ctl')
    attitude_ctl = AttitudeControl()
    attitude_ctl.run()

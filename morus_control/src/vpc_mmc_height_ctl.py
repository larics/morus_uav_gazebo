#!/usr/bin/env python

__author__ = 'thaus'

import rospy, math
from pid import PID
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32
from dynamic_reconfigure.server import Server
from morus_control.cfg import VpcMmcuavZCtlParamsConfig
from morus_msgs.msg import PIDController
from morus_msgs.msg import MotorSpeed
from mav_msgs.msg import Actuators


class HeightControl:
    '''
    Class implements ROS node for cascade (z, vz) PID control for MAV height.
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

        self.z_sp = 1.0                 # z-position set point
        self.z_ref_filt = 0             # z ref filtered
        self.z_mv = 0                   # z-position measured value
        self.pid_z = PID()              # pid instance for z control

        self.vz_sp = 0                  # vz velocity set_point
        self.vz_mv = 0                   # vz velocity measured value
        self.pid_vz = PID()             # pid instance for z-velocity control

        #########################################################
        #########################################################
        # Add parameters for z controller
        self.pid_z.set_kp(0.5)
        self.pid_z.set_ki(0.01)
        self.pid_z.set_kd(0.75)
        self.pid_z.set_lim_high(5)      # max vertical ascent speed
        self.pid_z.set_lim_low(-5)      # max vertical descent speed

        # Add parameters for vz controller
        self.pid_vz.set_kp(20.0)#87.2)
        self.pid_vz.set_ki(0.1)
        self.pid_vz.set_kd(10.0)#10.89)
        self.pid_vz.set_lim_high(350)   # max velocity of a motor
        self.pid_vz.set_lim_low(-350)   # min velocity of a motor
        #########################################################
        #########################################################

        self.mot_speed = 0              # referent motors velocity, computed by PID cascade

        self.t_old = 0

        rospy.Subscriber('pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('odometry', Odometry, self.vel_cb)
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)
        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('pid_vz', PIDController, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        #self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        #self.pub_gm_mot = rospy.Publisher('collectiveThrust', GmStatus, queue_size=1)       
        self.cfg_server = Server(VpcMmcuavZCtlParamsConfig, self.cfg_callback)
        self.rate = rospy.get_param('~rate', 10)
        self.ros_rate = rospy.Rate(self.rate)                 # attitude control at 100 Hz
        self.t_start = rospy.Time.now()

    def run(self):
        '''
        Runs ROS node - computes PID algorithms for z and vz control.
        '''

        while not self.start_flag and not rospy.is_shutdown():
            print 'Waiting for pose measurements.'
            rospy.sleep(0.5)
        print "Starting height control."

        self.t_old = rospy.Time.now()
        #self.t_old = datetime.now()

        while not rospy.is_shutdown():
            rospy.sleep(1.0/float(self.rate))

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
            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            #t = datetime.now()
            #dt = (t - self.t_old).total_seconds()
            if dt > 1.05/float(self.rate) or dt < 0.95/float(self.rate):
                #print dt
                pass

            self.t_old = t
            #                              (m_uav + m_arms)/(C*4)
            self.mot_speed_hover = math.sqrt(9.81*(30.0+1.0*4)/(0.000456874*4.0))
            # prefilter for reference
            #a = 0.1
            #self.z_ref_filt = (1-a) * self.z_ref_filt  + a * self.z_sp
            vz_ref = self.pid_z.compute(self.z_sp, self.z_mv, dt)
            #print "vz_ref", vz_ref
            self.mot_speed = self.mot_speed_hover + \
                        self.pid_vz.compute(vz_ref, self.vz_mv, dt)
            #print "mot_speed", self.mot_speed

            ########################################################
            ########################################################

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
        if not self.start_flag:
            self.start_flag = True
        self.z_mv = msg.pose.position.z

    def vel_cb(self, msg):
        '''
        Velocity callback (linear velocity - x,y,z)
        :param msg: Type Vector3Stamped
        '''
        #if not self.start_flag:
        #    self.start_flag = True
        self.vz_mv = msg.twist.twist.linear.z

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

    def cfg_callback(self, config, level):
        """
        Callback for dynamically reconfigurable parameters (P,I,D gains for height and velocity controller)
        """
        #print "CFG callback"

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

    rospy.init_node('vpc_mmcuav_z_controller')
    height_ctl = HeightControl()
    height_ctl.run()


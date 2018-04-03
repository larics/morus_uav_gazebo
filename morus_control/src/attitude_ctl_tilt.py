import rospy
import numpy as np
import math
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from morus_msgs.msg import PIDController
from std_msgs.msg import *
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from pid import PID


class HeightCtl():

    def __init__(self):
        """Initialize all needed variables for quadcopter attitude control.
        
        Subscribe to:
            /morus/pose         - used to extract z-position of UAV
            /morus/velocity     - used to extract vz position of UAV
            /morus/pos_ref      - usedd to set the reference for z-position
            /morus/vel_ref      - used to set the reference for vz-position 
            
        Publishes:
            /morus/mot_vel_ref  - referent value for thrust in terms of motor velocity (rad/s)
            /morus/pid_z        - publishes PID-z data - referent value, measured value, P, I, D and total component
            /morus/pid_vz       - publishes PID-vz data - referent value, measured value, P, I, D and total component
            
        Dynamic reconfigure is used to set controller params online """

        self.first_measurement = False             # indicates if we received first measurement

        # define PID for height control
        self.z_sp = 0
        self.z_ref_filt = 0
        self.z_mv = 0
        self.pid_z = PID()

        # define PID for height rate control
        self.vz_sp = 0
        self.vz_mv = 0
        self.pid_vz = PID()

        # Add parameters for z-height PID controller
        self.pid_z.set_kp(10)
        self.pid_z.set_ki(0.2)
        self.pid_z.set_kd(0.2)

        # Add parameters for vz-height PID controller
        self.pid_vz.set_kp(1)
        self.pid_vz.set_ki(0.2)
        self.pid_vz.set_kd(0.2)

        # set maximum ascend and descend vertical speed
        self.pid_z.set_lim_high(5)
        self.pid_z.set_lim_low(-5)

        # set maximum angular velocity for rotors
        self.pid_vz.set_lim_high(1000)
        self.pid_vz.set_lim_low(-1000)

        # referent motors velocity, computed by PID cascade

        self.t_old = 0

        # subscribers to certain topics
        rospy.Subscriber('/morus/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/morus/odomtery', Odometry, self.vel_cb)
        # pose and velocity ref? topic is not active
        rospy.Subscriber('vel_ref', Vector3, self.vel_ref_cb)
        rospy.Subscriber('pos_ref', Vector3, self.pos_ref_cb)

        # publishers
        self.pub_pid_z = rospy.Publisher('pid_z', PIDController, queue_size=1)
        self.pub_pid_vz = rospy.Publisher('pid_vz', PIDController, queue_size=1)
        self.mot_ref_pub = rospy.Publisher('mot_vel_ref', Float32, queue_size=1)
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.tilt_0_pub = rospy.Publisher('/morus/angle_tilt_0_controller/command', Float64, queue_size=1)
        self.tilt_1_pub = rospy.Publisher('/morus/angle_tilt_1_controller/command', Float64, queue_size=1)
        self.tilt_2_pub = rospy.Publisher('/morus/angle_tilt_2_controller/command', Float64, queue_size=1)
        self.tilt_3_pub = rospy.Publisher('/morus/angle_tilt_3_controller/command', Float64, queue_size=1)
        #self.tilt_stabilize = rospy.Publisher(/)
        self.ros_rate = rospy.Rate(10)
        self.t_start = rospy.Time.now()

    def pose_cb(self, msg):
        """
        Pose (6DOF - position and orientation) callback 
        :param msg: Type PoseWithCovarianceStamped
        """
        if not self.first_measurement:
            self.first_measurement=True
            self.z_mv = msg.pose.position.z


    def vel_cb(self, msg):
        """
        Velocity callback (linear velocity - x, y, z)
        :param msg: Type Vector3Stamped
        """
        self.vz_mv = msg.twist.twist.linear.z

    def vel_ref_cb(self, msg):
        """
        Referent velocity callback. Use received velocity values when during initial tuning
        velocity controller (i.e. when position controller still not implemented).
        :param msg: Type Vector3        
        """
        self.vz_sp = msg.z

    def pos_ref_cb(self, msg):
        """
        Referent position callback. Received value (z-component is  used as  a  referent height.        
        :param msg: Type Vector3
        """

    def run(self):
        """Run ros node, compute PID algorithms for z and vz control"""

        # init publisher
        while not self.first_measurement:
            print("Waiting for pose measurement")
            rospy.sleep(0.5)
        print("Start height control.")

        self.t_old = rospy.Time.now()

        while not rospy.is_shutdown():
            self.ros_rate.sleep()

            t = rospy.Time.now()
            dt = (t - self.t_old).to_sec()
            if dt > 0.105 or dt < 0.095:
                print(dt)

            self.t_old = t

            # publish angles for tilt (in order to have rotors in normal position)
            self.tilt_0_pub.publish(0.0)
            self.tilt_1_pub.publish(0.0)
            self.tilt_2_pub.publish(0.0)
            self.tilt_3_pub.publish(0.0)

            self.motor_hover_speed = math.sqrt(293/0.000456874/4)
            # prefilter for reference (Why we use prefilter for reference)
            a = 0.1
            self.z_ref_filt = (1-a) * self.z_ref_filt + a * self.z_sp
            vz_ref = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
            self.motor_speed = self.motor_hover_speed + \
                               self.pid_vz.compute(vz_ref, self.vz_mv, dt)

            motor_speed_msg = Actuators()
            motor_speed_msg.angular_velocities = [self.motor_speed, self.motor_speed,
                                                  self.motor_speed, self.motor_speed]
            self.pub_mot.publish(motor_speed_msg)
            # Publish PID data - could be useful for tuning
            self.pub_pid_z.publish(self.pid_z.create_msg())
            self.pub_pid_vz.publish(self.pid_vz.create_msg())

            self.z_sp = 1



if __name__ == "__main__":
    rospy.init_node('mav_z_controller')
    height_ctl = HeightCtl()
    height_ctl.run()
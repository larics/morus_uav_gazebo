import rospy
import numpy
import math
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, TwistStamped
from morus_msgs.msg import PIDController
from std_msgs.msg import *
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from pid import PID



class AngleTiltCtl:

    def __init__(self):
        self.first_measurement = False
        self.t = 0
        self.t_old = 0
        # initialize subscribers
        self.pose_sub = rospy.Subscriber('/morus/pose', PoseStamped, self.pose_cb)
        self.odometry = rospy.Subscriber('/morus/odometry', Odometry, self.odometry_cb)
        # initialize publishers
        self.pub_mot = rospy.Publisher('/gazebo/command/motor_speed', Actuators, queue_size=1)
        self.ros_rate = rospy.Rate(50)

        self.z_sp = 0                           # z-position set point
        self.z_ref_filt = 0                     # z ref filtered
        self.z_mv = 0                           # z-position measured value
        self.pid_z = PID()                      # pid instance for z control

        self.vz_sp = 0                          # vz velocity set_point
        self.vz_mv = 0                          # vz velocity measured value
        self.pid_vz = PID()                     # pid instance for z-velocity control

        self.euler_mv = Vector3(0, 0, 0)        # measured euler angles
        self.euler_sp = Vector3(0, 0, 0)        # euler angles referent values
        self.euler_rate_mv = Vector3(0, 0, 0)   # measured angular velocities
        self.dwz = 0

        ########################################
        ########################################
        # TO DO: try to implement height and angle control in different python scripts

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
        self.pid_z.set_kp(2)
        self.pid_z.set_ki(0.5)
        self.pid_z.set_kd(2)

        # Add parameters for vz-height PID controller
        self.pid_vz.set_kp(20)
        self.pid_vz.set_ki(0)
        self.pid_vz.set_kd(0)

        # set maximum ascend and descend vertical speed
        self.pid_z.set_lim_high(5)
        self.pid_z.set_lim_low(-5)

        # set maximum angular velocity for rotors
        self.pid_vz.set_lim_high(1000)
        self.pid_vz.set_lim_low(-1000)

        ########################################
        ########################################

        # initialize pitch_rate PID controller
        self.pitch_rate_PID = PID()
        self.pitch_rate_mv = 0

        # initialize roll rate PID controller
        self.roll_rate_PID = PID()
        self.roll_rate_mv = 0

        # initialize yaw rate PID controller
        self.yaw_rate_PID = PID()
        self.yaw_rate_mv = 0

        # set PID P, I, D gains for PIDs
        self.pitch_rate_PID.set_kp(75)
        self.pitch_rate_PID.set_ki(15)
        self.pitch_rate_PID.set_kd(2)

        self.roll_rate_PID.set_kp(75)
        self.roll_rate_PID.set_ki(15)
        self.roll_rate_PID.set_kd(2)

        self.yaw_rate_PID.set_kp(5)
        self.yaw_rate_PID.set_ki(0)
        self.yaw_rate_PID.set_kd(0)


    def pose_cb(self, msg):
        """
        Callback functon for assigning values from pose IMU
                
        :param msg: /morus/pose PoseStamped msg type used to extract pose and orientation of UAV
         
        """
        # entered subscribed callback func, set first_measurement flag as True
        self.first_measurement = True

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w


    def odometry_cb(self, msg):
        """
        Callback function for assigning values from odometry measurements 
        
        :param msg: nav_msgs/Odometry,
        an estimate of position and velocity in free space         
        """

        self.lin_x = msg.twist.twist.linear.x
        self.lin_y = msg.twist.twist.linear.y
        self.lin_z = msg.twist.twist.linear.z

        self.roll_rate_mv = msg.twist.twist.angular.x
        self.pitch_rate_mv = msg.twist.twist.angular.y
        self.yaw_rate_mv = msg.twist.twist.angular.z

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

            self.t_old = t
            self.quat_to_eul_conv(self.qx, self.qy, self.qz, self.qw)
            print(self.roll, self.pitch, self.yaw)

            self.hover_speed = math.sqrt(293/0.000456874/4)

            # change in motor speed produced by changing angle reference
            dwz = self.yaw_rate_PID.compute(self.euler_sp.z, self.euler_mv.z, dt)
            dwy = self.pitch_rate_PID.compute(self.euler_sp.y, self.euler_mv.y, dt)
            dwx = self.roll_rate_PID.compute(self.euler_sp.x, self.euler_mv.x, dt)

            # change in motor speed caused by height reference
            a = 0.1
            self.z_ref_filt = (1 - a) * self.z_ref_filt + a * self.z_sp
            vz_ref = self.pid_z.compute(self.z_ref_filt, self.z_mv, dt)
            domega_z = self.pid_vz.compute(vz_ref, self.vz_mv, dt)
            # domega_z = 0

            # motor speeds calculated with respect to height and angle control
            #if self.z_mv > self.z_sp:
                #domega_z = - domega_z
            #else:
                #pass
            print(self.z_sp, self.z_mv)
            motor_speed_1 = self.hover_speed + domega_z + dwz - dwy
            motor_speed_2 = self.hover_speed + domega_z - dwz + dwx
            motor_speed_3 = self.hover_speed + domega_z + dwz + dwy
            motor_speed_4 = self.hover_speed + domega_z - dwz - dwx
            print(motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4)

            # publish motor speeds
            motor_speed_msg = Actuators()
            motor_speed_msg.angular_velocities = [motor_speed_1, motor_speed_2,
                                                  motor_speed_3, motor_speed_4]

            self.pub_mot.publish(motor_speed_msg)
            self.z_sp = 2
            self.z_mv = self.z


if __name__ == "__main__":
    rospy.init_node('mav_angles_controller')
    angle_ctl = AngleTiltCtl()
    angle_ctl.run()




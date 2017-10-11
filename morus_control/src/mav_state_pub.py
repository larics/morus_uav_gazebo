#!/usr/bin/env python

__author__ = 'thaus'

import rospy
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped, Pose, Vector3Stamped
from sensor_msgs.msg import Imu
import math
import numpy

class VelocityPublisher:

    def __init__(self):
        self.time_old = rospy.Time.now()
        self.pose_old = Pose()
        self.start_flag = False
        self.vel_msg = TwistStamped()
        self.euler = Vector3Stamped()

        self.vel_pub = rospy.Publisher('velocity', TwistStamped, queue_size=1)
        self.euler_pub = rospy.Publisher('euler', Vector3Stamped, queue_size=1)
        rospy.Subscriber('pose_with_covariance', PoseWithCovarianceStamped, self.pose_gt_cb)
        rospy.Subscriber('imu', Imu, self.imu_cb)
        rospy.sleep(0.1)

    def run(self):

        while not rospy.is_shutdown():
            rospy.spin()

    def pose_gt_cb(self, msg):
        if self.start_flag == False:
            self.time_old = msg.header.stamp
            self.pose_old = msg.pose.pose
            self.start_flag = True
        else:
            noise = numpy.random.normal(0, 0.02, 3)  # mean=0, std.dev=0.02, 3 values
            dt = (msg.header.stamp - self.time_old).to_sec() # elapsed time since last message in seconds
            self.vel_msg.twist.linear.x = (msg.pose.pose.position.x - self.pose_old.position.x) / dt + noise[0]
            self.vel_msg.twist.linear.y = (msg.pose.pose.position.y - self.pose_old.position.y) / dt + noise[1]
            self.vel_msg.twist.linear.z = (msg.pose.pose.position.z - self.pose_old.position.z) / dt + noise[2]
            self.vel_msg.header.stamp = rospy.Time.now()
            self.vel_pub.publish(self.vel_msg)
            self.time_old = msg.header.stamp
            self.pose_old = msg.pose.pose

    def imu_cb(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # conversion quaternion to euler (yaw - pitch - roll)
        self.euler.vector.x = math.atan2(2 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
        self.euler.vector.y = math.asin(2 * (qw * qy - qx * qz))
        self.euler.vector.z = math.atan2(2 * (qw * qz + qx * qy), qw * qw + qx * qx - qy * qy - qz * qz)
        self.euler.header = msg.header
        self.euler_pub.publish(self.euler)

if __name__ == '__main__':

    rospy.init_node('mav_vel_publisher')
    vel_pub = VelocityPublisher()
    print "Starting publishing"
    vel_pub.run()

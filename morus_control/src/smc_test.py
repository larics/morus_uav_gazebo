#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3

if __name__ == "__main__":

    rospy.init_node('smc_test', anonymous=True)

    pos_pub = rospy.Publisher('pos_ref', Vector3, queue_size=1)
    angle_pub = rospy.Publisher('angle_ref', Vector3, queue_size=1)

    pos_msg = Vector3(0, 0, 1)
    angle_msg = Vector3(0, 0, 0)

    try:

        # Go to 2m height
        rospy.sleep(15)
        pos_msg.z = 2
        pos_pub.publish(pos_msg)

        # First cross coupling disturbance
        rospy.sleep(10)
        angle_msg.x = 0.1
        angle_pub.publish(angle_msg)
        rospy.sleep(10)
        angle_msg.x = 0
        angle_pub.publish(angle_msg)

        # Second cross coupling disturbance
        rospy.sleep(10)
        angle_msg.y = 0.1
        angle_pub.publish(angle_msg)
        rospy.sleep(10)
        angle_msg.y = 0
        angle_pub.publish(angle_msg)

        # Both disturbances at the same time
        rospy.sleep(10)
        angle_msg.y = 0.1
        angle_msg.x = 0.1
        angle_pub.publish(angle_msg)
        rospy.sleep(10)
        angle_msg.y = 0
        angle_msg.x = 0
        angle_pub.publish(angle_msg)

    except rospy.ROSInterruptException:
        pass

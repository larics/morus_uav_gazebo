#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy

from std_msgs.msg import Int8

def saturation(value, min, max):
    if (value < min):
        return min  
    elif (value > max):
        return max
    else:
        return value

class Commander():
    #carrot following controller for z axis requires knowing the position of the UAV 
    
    MIN_HEIGHT = 0.7
    MAX_HEIGHT = 2.5

    def __init__(self):
        # Create a publisher for roll pitch yaw cmnds
        self.UAV_pose = PointStamped()
        self.pub = rospy.Publisher('morus/euler_ref', Vector3, queue_size=1)
        self.pub2 = rospy.Publisher('morus/pos_ref', Vector3, queue_size=1)
        # Initialize message variables.

        # Create a subscriber for color msg
        self.game_status = -1
        rospy.Subscriber("/game_loop/running", Int8, self.game_status_callback)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("morus/position", PointStamped, self.position_callback)
        rate = rospy.Rate(1) # 1hz
        # Main while loop.
        while not rospy.is_shutdown():
            rate.sleep()

    def game_status_callback(self, msg):
        self.game_status = msg.data

    # Subscribe to teleop msgs
    def cmd_vel_callback(self,data):
        
        # Don't do anything if game is not running
        if not self.game_status == 2:
            return

        cmd_roll_pitch_yaw = Vector3()
        cmd_roll_pitch_yaw.x=data.linear.x
        cmd_roll_pitch_yaw.y=data.linear.y
        cmd_roll_pitch_yaw.z=data.angular.z
        self.pub.publish(cmd_roll_pitch_yaw)

        cmd_position = Vector3()
        cmd_position.z = self.UAV_pose.point.z+10*data.linear.z #5ms for 20Hz
        cmd_position.z = saturation(cmd_position.z, Commander.MIN_HEIGHT, Commander.MAX_HEIGHT)
        self.pub2.publish(cmd_position)

    def position_callback(self,data):
        self.UAV_pose = data
    

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('morusCommander')
    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ne = Commander()
    except rospy.ROSInterruptException:
        pass
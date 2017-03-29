#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from morus_uav_ros_msgs.msg import GmStatus

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
    pub = rospy.Publisher('collectiveThrust', GmStatus, queue_size=10)
    colThrust = GmStatus()
    colThrust.motor_id = 5
    colThrust.force_M = data.data
    pub.publish(colThrust)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('trhurst_listener', anonymous=True)

    rospy.Subscriber("mot_vel_ref", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
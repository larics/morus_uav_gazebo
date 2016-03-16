#!/usr/bin/env python

__author__ = 'bmaric'

import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64

class Simulator2Model():
    def __init__(self):

        #Initialization of the class
        self.min_pose_dist = -0.2 #min in simulator
        self.min_pose_angle = 0.9 #min angle of dynamixel
        self.max_pose_dist = 0.2
        self.max_pose_angle = -2.6

        #set limits
        self.min_angle = -2.55
        self.max_angle = 0.75

        #calculate coefs for angle=k*pose + l 
        #y=-8.75*x - 0.85
        self.k = (self.max_pose_angle - self.min_pose_angle)/(self.max_pose_dist - self.min_pose_dist)
        self.l = self.min_pose_angle - self.min_pose_dist*(self.max_pose_angle - self.min_pose_angle)/(self.max_pose_dist - self.min_pose_dist)



        self.temp_pose_dist = 0

        #Init subscribers
        rospy.Subscriber('/morus/movable_mass_0_position_controller/state', JointControllerState, self.mass_pose_cb)

        #Init publishers
        self.pub_model_mass = rospy.Publisher('/Joint_mass_controller/command', Float64, queue_size=1)


    def mass_pose_cb(self, msg):
        #Read mass position in simulator
        
        
        self.temp_pose_dist = msg.set_point

        #Publish mass position to model
        temp_mass_angle = self.k*self.temp_pose_dist + self.l

        if temp_mass_angle < self.min_angle :
            temp_mass_angle = self.min_angle
        elif temp_mass_angle > self.max_angle:
            temp_mass_angle = self.max_angle

        print temp_mass_angle

        #publish
        self.pub_model_mass.publish(temp_mass_angle)
        

    def run(self):

        try:
            while not rospy.is_shutdown():
                pass
        except rospy.ROSInterruptException:
            pass




if __name__ == '__main__':

    rospy.init_node('simulator2model')
    sim2mod = Simulator2Model()

    try:
        sim2mod.run()
    except rospy.ROSInterruptException:
        pass

    


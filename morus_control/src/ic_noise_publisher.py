#!/usr/bin/env python
import rospy
import os
import random
from std_msgs.msg import Float64MultiArray

class NoisePublisher:
	def __init__(self):
		self.noise_pub = rospy.Publisher('motor_noise', Float64MultiArray, queue_size=1)
		self.ros_rate = rospy.Rate(18)

		self.ros_msg = Float64MultiArray()
		self.ros_msg.data.append(0)
		self.ros_msg.data.append(0)
		self.ros_msg.data.append(0)
		self.ros_msg.data.append(0)
		self.noise = []

	def run(self):
		index_front = random.randint(0, len(self.noise)-1)
		index_back = random.randint(0, len(self.noise)-1)
		index_left = random.randint(0, len(self.noise)-1)
		index_right = random.randint(0, len(self.noise)-1)
		while not rospy.is_shutdown():
			
			self.ros_msg.data[0] = self.noise[index_front]
			self.ros_msg.data[1] = self.noise[index_right]
			self.ros_msg.data[2] = self.noise[index_back]
			self.ros_msg.data[3] = self.noise[index_left]

			index_front = index_front + 1
			index_right = index_right + 1
			index_back = index_back + 1
			index_left = index_left + 1

			if (index_front >= len(self.noise)):
				index_front = 0;
			if (index_right >= len(self.noise)):
				index_right = 0;
			if (index_back >= len(self.noise)):
				index_back = 0;
			if (index_left >= len(self.noise)):
				index_left = 0;

			self.noise_pub.publish(self.ros_msg)

			self.ros_rate.sleep()

	def load_noise(self, path):
		noise_dat = open(path,'r')
		for line in noise_dat:
			self.noise.append(float(line)*2*3.14/60)
		noise_dat.close()

if __name__ == '__main__':
	rospy.init_node('noise_publisher')
	noise = NoisePublisher()
	noise.load_noise("/home/suiauthon/morus_ws/src/morus_uav_gazebo/morus_control/config/noise.txt")
	noise.run()
#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from math import sqrt
import time

class GameNode():
	"""
	This class represents the game node loop. It will calculate the distance
	from the goal, and measure time from the moment the UAV starts moving.
	"""

	TARGET_X = -17.2652
	TARGET_Y = -24.7699

	def __init__(self):
		
		rospy.Subscriber("morus/position", PointStamped, self.position_callback)
		self.distance_pub = rospy.Publisher("game_loop/distance", Float64, queue_size=1)
		self.time_pub = rospy.Publisher("game_loop/elapsed_time", Float64, queue_size=1)
		
		self.first_post = False
		self.started_moving = False

		self.initial_x = 0
		self.initial_y = 0
		self.x_mv = 0
		self.y_mv = 0

		self.elapsed_time = 0
		self.last_time = -1 


	def position_callback(self, data):

		# Check if first callback, record initial position
		if not self.first_post:
			self.first_post = True
			self.initial_x = data.point.x
			self.initial_y = data.point.y

		self.x_mv = data.point.x
		self.y_mv = data.point.y

		#  Check if started moving
		if not self.started_moving:
			
			init_dist = sqrt( 
				(self.x_mv - self.initial_x)**2 +  
				(self.y_mv - self.initial_y)**2 ) 
			if init_dist > 1e-1:
				self.started_moving = True
				self.last_time = time.time()

		else:

			temp_time = time.time()
			self.elapsed_time += temp_time - self.last_time
			self.last_time = temp_time

			timeMsg = Float64()
			timeMsg.data = self.elapsed_time
			self.time_pub.publish(timeMsg)

			# Publish distance to goal
			distance = sqrt( 
				(self.x_mv - GameNode.TARGET_X)**2 +  
				(self.y_mv - GameNode.TARGET_Y)**2 ) 
			newMsg = Float64()
			newMsg.data = distance
			self.distance_pub.publish(newMsg)


if __name__ == '__main__':
	rospy.init_node("game_loop_node")
	game_node = GameNode()
	rospy.spin()
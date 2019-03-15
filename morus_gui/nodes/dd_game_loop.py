#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, Int8
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

from math import sqrt
import time

class GameNode():
	"""
	This class represents the game node loop. It will calculate the distance
	from the goal, and measure time from the moment the UAV starts moving.
	"""

	TARGET_X = -17.2652
	TARGET_Y = -24.7699

	NOT_RUNNING = 1
	RUNNING = 2
	FINISHED = 3

	INIT_TOL = 1e-1
	TARGET_TOL = 2

	def __init__(self):
		
		self.status = GameNode.NOT_RUNNING

		self.first_post = False
		self.started_moving = False
		self.game_running = False
		
		self.initial_x = 0
		self.initial_y = 0
		self.initial_z = 0
		self.x_mv = 0
		self.y_mv = 0

		self.elapsed_time = 0
		self.last_time = -1 
		
		rospy.Subscriber("morus/position", PointStamped, self.position_callback)
		rospy.Subscriber("morus/imu", Imu, self.imu_callback)
		rospy.Subscriber("morus/velovity", TwistStamped, self.vel_callback)
		self.game_status = rospy.Publisher("game_loop/running", Int8, queue_size=1) 
		self.distance_pub = rospy.Publisher("game_loop/distance", Float64, queue_size=1)
		self.time_pub = rospy.Publisher("game_loop/elapsed_time", Float64, queue_size=1)
		

	def vel_callback(self, data):
		lx = data.twist.linear.x
		ly = data.twist.linear.y
		lz = data.twist.linear.z

		l = sqrt(lx**2 + ly**2 + lz**2)
		if l > 5:
			self.game_running = False

	def imu_callback(self,data):
		ax = data.angular_velocity.x
		ay = data.angular_velocity.y
		az = data.angular_velocity.z

		# In case UAV becomes unstable
		a = sqrt(ax**2 + ay**2 + az**2)
		if  a > 5:
			self.game_running = False

	def position_callback(self, data):

		# Check if first callback, record initial position
		if not self.first_post:
			self.first_post = True
			self.initial_x = data.point.x
			self.initial_y = data.point.y
			self.initial_z = data.point.z

		self.x_mv = data.point.x
		self.y_mv = data.point.y
		self.z_mv = data.point.z

		
		if not self.started_moving:
			
			#  Check if started moving	
			init_dist = sqrt( 
				(self.x_mv - self.initial_x)**2 +  
				(self.y_mv - self.initial_y)**2 + 
				(self.z_mv - self.initial_z)**2 ) 
			if init_dist > GameNode.INIT_TOL:
				self.started_moving = True
				self.game_running = True
				self.last_time = time.time()

			else:
				self.pub_game_status(GameNode.NOT_RUNNING)


		elif self.game_running:

			# Record elapsed time
			temp_time = time.time()
			self.elapsed_time += temp_time - self.last_time
			self.last_time = temp_time

			# Publish elapsed time
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

			# Stop the game if target is reached
			if distance < GameNode.TARGET_TOL:
				self.game_running = False

			self.pub_game_status(GameNode.RUNNING)

		else:
			# Finish the game by stopping the siulation
			self.pub_game_status(GameNode.FINISHED)
			
	def pub_game_status(self, status):
		gameMsg = Int8()
		gameMsg.data = status
		self.game_status.publish(gameMsg)

if __name__ == '__main__':
	rospy.init_node("game_loop_node")
	game_node = GameNode()
	
	print("GameLoopNode: Starting")
	while not game_node.status == GameNode.FINISHED:
		rospy.sleep(0.01)

	print("GameLoopNode: Pausing Simulation")
	service_call = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
	service_call()	
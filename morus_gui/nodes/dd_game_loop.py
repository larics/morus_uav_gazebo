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
    TARGET_TOL = 3

    MAX_ANGULAR_VEL = 15
    MAX_LINEAR_VEL = 8

    def __init__(self):

        self.status = GameNode.NOT_RUNNING

        self.first_position_set = False
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

    def checkIfMoving(self):
        """
        Check if the UAV started moving. If it started moving do nothing,
        otherwise set game flags to true.
        """

        # Do nothing if it already is moving or first position not yet set
        if self.started_moving or not self.first_position_set:
            return 

        # Calculate from initial distance
        init_dist = sqrt( 
            (self.x_mv - self.initial_x)**2 +  
            (self.y_mv - self.initial_y)**2 + 
            (self.z_mv - self.initial_z)**2 ) 

        if init_dist > GameNode.INIT_TOL:
            # If movement is detected raise game flags
            print("GameNode: Game start is detected")
            self.started_moving = True
            self.game_running = True
            self.last_time = time.time()
            self.pub_game_status(GameNode.RUNNING)

        else:
            print("GameNode: Game start not detected")
            self.pub_game_status(GameNode.NOT_RUNNING)

    def gameRunningActivities(self):
        """
        Perform all activities when game is running
        """
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
            print("GameNode: Distance reached, game is finished")
            self.game_running = False
            self.pub_game_status(GameNode.FINISHED)
        else:
            self.pub_game_status(GameNode.RUNNING)

    def run_once(self):
        """
        Run game loop once.
        """

        # UAV is not moving, game not started
        if not self.started_moving and not self.game_running:
            self.checkIfMoving()

        # UAV is moving, and game started 
        elif self.started_moving and self.game_running:
            self.gameRunningActivities()

        # UAV is moving and game is finished - detect game as finished
        elif self.started_moving and not self.game_running:
            print("GameNode - MainLoop: Game deteced as finished")
            self.game_running = False
            self.pub_game_status(GameNode.FINISHED)

        # Otherwise game states are wrong - still finish the game
        else:
            print("GameNode - MainLoop: Something went wrong, stopping the game...")
            self.game_running = False
            self.started_moving = False
            self.pub_game_status(GameNode.FINISHED)

    def vel_callback(self, data):
        lx = data.twist.linear.x
        ly = data.twist.linear.y
        lz = data.twist.linear.z

        l = sqrt(lx**2 + ly**2 + lz**2)
        if l > GameNode.MAX_LINEAR_VEL  and self.game_running:
            print("GameNode: Linear velocity too high {}".format(l))
            self.game_running = False
            self.pub_game_status(GameNode.FINISHED)

    def imu_callback(self,data):
        ax = data.angular_velocity.x
        ay = data.angular_velocity.y
        az = data.angular_velocity.z

        # In case UAV becomes unstable
        a = sqrt(ax**2 + ay**2 + az**2)
        if a > GameNode.MAX_ANGULAR_VEL and self.game_running:
            print("GameNode: Angular velocity too high {}".format(a))
            self.game_running = False
            self.pub_game_status(GameNode.FINISHED)

    def position_callback(self, data):

        # Check if first callback, record initial position
        if not self.first_position_set:
            self.first_position_set = True
            self.initial_x = data.point.x
            self.initial_y = data.point.y
            self.initial_z = data.point.z

        self.x_mv = data.point.x
        self.y_mv = data.point.y
        self.z_mv = data.point.z

    def pub_game_status(self, status):
        """
        Publish given game status and save it to a class variable
        """

        gameMsg = Int8()
        gameMsg.data = status
        self.game_status.publish(gameMsg)
        self.status = status

    def pauseSimulation(self):
        """
        Send request to pause simulation
        """

        print("GameLoopNode: Pausing Simulation")
        service_call = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        service_call()  

if __name__ == '__main__':
    rospy.init_node("game_loop_node")
    game_node = GameNode()

    print("GameNode: Starting")
    while not game_node.status == GameNode.FINISHED and not rospy.is_shutdown():
        rospy.sleep(0.1)
        game_node.run_once()

    print("GameNode: Game finished")
    counter = 0
    while counter < 20 and not rospy.is_shutdown():
        game_node.pub_game_status(GameNode.FINISHED)
        rospy.sleep(0.1)
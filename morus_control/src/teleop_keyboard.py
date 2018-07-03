#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys, select, termios, tty

#modified teleop_twist_keyboard.py 
__author__ = 'bmaric'

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
roll-pitch [rad]:
   u    i    o
   j    k    l
   m    ,    .

yaw [rad]:

	g 	h

altitude:
	r - increase
	f - decrease
	v - land

movable mass(counter clock):
mass0(red) 1-in, 2-out
mass1(blue) 3-in, 4-out
mass2(blue) 5-in, 6-out
mass3(blue) 7-in, 8-out
reset all - 9

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

			# (roll, pitch, yaw, alt)

moveBindings = {
		'i':(-1,0,0,0), #minus roll
		'o':(-1,1,0,0), #minus rol, plus pitch
		'j':(0,-1,0,0),	#minus pitch
		'l':(0,1,0,0), #plus pitch
		'u':(-1,-1,0,0), #minus roll, minus pitch
		',':(1,0,0,0), #plus roll
		'.':(1,1,0,0), #plus roll, plus pitch
		'm':(1,-1,0,0), #plus roll, minus pitch
		'r':(0,0,0,1), #plus alt
		'f':(0,0,0,-1), #minus alt
		'v':(0,0,0,0), #land
		'g':(0,0,1,0), #plus yaw
		'h':(0,0,-1,0) #minus yaw

	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'e':(1,1.1),
		'c':(1,.9),
		'w':(1.1,1),
		'x':(.9,1),
	      }

massBindings = {
		'1':(-1,0,0,0), #mass0 in
		'2':(1,0,0,0), #mass0 out
		'3':(0,-1,0,0), #mass1 in
		'4':(0,1,0,0), #mass1 out
		'5':(0,0,-1,0), #mass2 in
		'6':(0,0,1,0), #mass2 out
		'7':(0,0,0,-1), #mass3 in
		'8':(0,0,0,1), #mass3 out
		'9':(0,0,0,0) #mass zero
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .5
turn = 0.05

def vels(speed,turn):
	return "currently:\tlinear step %s\tangular %s " % (speed,turn)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/morus/cmd_vel', Twist)
	pub_mass0 = rospy.Publisher('/morus/movable_mass_0_position_controller/command', Float64)
	pub_mass1 = rospy.Publisher('/morus/movable_mass_1_position_controller/command', Float64)
	pub_mass2 = rospy.Publisher('/morus/movable_mass_2_position_controller/command', Float64)
	pub_mass3 = rospy.Publisher('/morus/movable_mass_3_position_controller/command', Float64)

	rospy.init_node('teleop_twist_keyboard')

	roll = 0
	pitch = 0
	yaw = 0
	alt = 0

	mass_0 = 0
	mass_1 = 0
	mass_2 = 0
	mass_3 = 0

	
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				roll = moveBindings[key][0]
				pitch = moveBindings[key][1]
				yaw += moveBindings[key][2]
				if moveBindings[key] == (0,0,0,0):
					alt = 0
				else:
					alt += moveBindings[key][3]

			elif key in massBindings.keys():
				mass_0 += massBindings[key][0]*0.05
				mass_1 += massBindings[key][1]*0.05
				mass_2 += massBindings[key][2]*0.05
				mass_3 += massBindings[key][3]*0.05

				if key == '9':
					mass_0 = 0
					mass_1 = 0
					mass_2 = 0
					mass_3 = 0

				
				
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print vels(speed,turn)
				if (status == 2):
					print msg
				status = (status + 1) % 3
			else:
				roll = 0
				pitch = 0
				yaw = 0 
				if (key == '\x03'):
					break

			twist = Twist()
			#twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = alt
			#twist.angular.x = roll; twist.angular.y = pitch; twist.angular.z = yaw
			twist.linear.x = 0*speed; twist.linear.y = 0*speed; twist.linear.z = alt*speed
			twist.angular.x = roll*turn; twist.angular.y = pitch*turn; twist.angular.z += yaw*turn
			pub.publish(twist)

			pub_mass0.publish(mass_0)
			pub_mass1.publish(mass_1)
			pub_mass2.publish(mass_2)
			pub_mass3.publish(mass_3)

	except:
		print 'greska'

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


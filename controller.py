#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ HB_1132]
# Author List:	[Shashwat Patel, Ankit Ahirwar, Arunesh Kumar Sagar, Ayush Tripathi]
# Filename:		feedback.py
# Functions: def main(),def signal_handler(sig, frame),def cleanup(),def task2_goals_Cb(msg),def aruco_feedback_Cb(msg),def inverse_kinematics(vel_x, vel_y, vel_z),def GTG(currentPos, desiredPos)
#			
# Nodes: Publishing Node:right_wheel_pub,left_wheel_pub,front_wheel_pub	
# Subscribing node = detected_aruco ,task2_goals

################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = [50,350,50,250,250]
y_goals = [350,50,50,350,50]
theta_goals = [0, 0, 0, 0, 0]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

hola_x = 0 
hola_y = 0 
hola_theta = 0

v = 10
w = 10
max_v = 5
Kxy = 0.755
Kw =  2.750 

##################### FUNCTION DEFINITIONS #######################

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	global v1, v2, v3
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()
	v1.force.x = 0
	v2.force.x = 0
	v3.force.x = 0
	right_wheel_pub.publish(v2)
	left_wheel_pub.publish(v1)
	front_wheel_pub.publish(v3)
	


def task2_goals_Cb(msg):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)
 
def aruco_feedback_Cb(msg):
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta
        
def inverse_kinematics(vel_x, vel_y, vel_z):
	v_1 = (-vel_x/2.0 + math.sqrt(3)*vel_y/2.0 - vel_z)
	v_2 = -vel_x/2.0 - math.sqrt(3)*vel_y/2.0 - vel_z
	v_3 = vel_x - vel_z
	return v_1, v_2, v_3

def GTG(currentPos, desiredPos):
	global Kxy, Kw
	# Finding the error in global frame
	err_x = desiredPos[0] - currentPos[0]
	err_y = currentPos[1] - desiredPos[1]
	error_t = desiredPos[2] - currentPos[2]
    # Implementing the P controller
	gain_x = Kxy*err_x
	gain_y = Kxy*err_y
	# Velocities in robot body frame
	if math.sqrt(err_x*err_x + err_y*err_y) > 0.048 or abs(error_t) > 0.048:
		vel_x  = v*(gain_x*math.cos(hola_theta) + gain_y*math.sin(hola_theta))
		vel_y  = v*(-gain_x*math.sin(hola_theta) + gain_y*math.cos(hola_theta))
		vel_z  = Kw*w*error_t
		v_1, v_2, v_3 = inverse_kinematics(vel_x, vel_y, vel_z)	
		return v_1, v_2, v_3, False
	else :
		return 0, 0, 0, True

def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	next_goal = 0
	x_d=x_goals[next_goal]
	y_d=y_goals[next_goal] 
	t_d=theta_goals[next_goal]	

	v1 = Wrench() 
	v2 = Wrench()
	v3 = Wrench()


	while not rospy.is_shutdown():

		v_1, v_2, v_3, reached = GTG([hola_x, hola_y, hola_theta], [x_d, y_d, t_d])
		v1.force.x = v_1
		v2.force.x = v_2
		v3.force.x = v_3
		right_wheel_pub.publish(v2)
		left_wheel_pub.publish(v1)
		front_wheel_pub.publish(v3)

		if reached:
			# If reached
			rospy.sleep(1)
			# Loading next goal pose
			next_goal = next_goal+1            
			if next_goal > len(x_goals)-1:
				next_goal = 0
			x_d = x_goals[next_goal]
			y_d = y_goals[next_goal]
			t_d = theta_goals[next_goal]

		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

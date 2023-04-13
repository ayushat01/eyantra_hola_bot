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
# Author List:		[Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar]
# Filename:		controller.py
# Functions:
#			[signal_handler, cleanup, task2_goals_Cb, limit_v, aruco_feedback_Cb, inverse_kinematics, GTG, main]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import math		# If you find it useful

import socket
from time import sleep
import signal		
import sys	

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

# test cases 
x_goals = [350, 150, 150, 350] # 296, 38, 
y_goals = [300, 300, 150, 150] # 70, 65, 
theta_goals = [PI/4, 3*PI/4, -3*PI/4, -PI/4]

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

hola_x = 0 
hola_y = 0 
hola_theta = 0

v = 2
w = 4
x = 1.1
max_v = 600

Kxy = 0.80
Kw =  8

linear_tol = 2
angular_tol = 0.05

ip = "192.168.43.13" 

##################### FUNCTION DEFINITIONS #######################

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	msg = f"{0} {0} {0}"
	conn.send(str.encode(msg))
	s.close()

def limit_v(v_1, v_2, v_3, x):
	while abs(v_1) > max_v or abs(v_2) > max_v or abs(v_3) > max_v:
		v_1 = v_1/x
		v_2 = v_2/x
		v_3 = v_3/x
	return int(v_1), int(v_2), int(v_3)


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
	v_1, v_2, v_3 = limit_v(v_1, v_2, v_3, x)
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
	# print(f"err {err_x} {err_y} {error_t}")
	if math.sqrt(err_x*err_x + err_y*err_y) > linear_tol or abs(error_t) > angular_tol:
		vel_x  = v*(gain_x*math.cos(hola_theta) + gain_y*math.sin(hola_theta))
		vel_y  = v*(-gain_x*math.sin(hola_theta) + gain_y*math.cos(hola_theta))
		vel_z  = Kw*w*error_t
		v_1, v_2, v_3 = inverse_kinematics(vel_x, vel_y, vel_z)	
		return v_1, v_2, v_3, False
	else :
		print("reached")
		return 0, 0, 0, True

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((ip, 8002))
	s.listen()
	conn, addr = s.accept()

def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)
	
	rate = rospy.Rate(100)

	next_goal = 0
	x_d=x_goals[next_goal]
	y_d=y_goals[next_goal] 
	t_d=theta_goals[next_goal]	

	while not rospy.is_shutdown():

		v_1, v_2, v_3, reached = GTG([hola_x, hola_y, hola_theta], [x_d, y_d, t_d])

		msg = f"{-v_3} {-v_2} {-v_1}"
		conn.send(str.encode(msg))
		# print(f"msg sent {msg}")
		data = conn.recv(1024) # Two way hand shake from esp.

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

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
from geometry_msgs.msg import PoseArray # Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from std_msgs.msg import String, Int32

import math		# If you find it useful

import socket
import time
import signal		
import sys	
import cv2
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles
from cv_basics.msg import aruco_data

################## GLOBAL VARIABLES ######################
PI = 3.14

right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None

hola_x = 0 
hola_y = 0 
hola_theta = 0

image = "/home/shashwat/ros_ws/src/hola_bot/scripts/smile.png"
mode = "Image" # "Image" for image mode, "function" for function mode
###################### Velocities ######################################
v = 3 # Linear velocity.
w = 4 # Angular velocity.
x = 1.1 # divider for limiting the each wheel velocity.
max_v = 300 # Maximum velocity of each wheel.
###########################################################################

skiping_width = 12 

###################### controller constants ################################

Kxy = 0.780 # Kp(v)
Kw =  10 # Kp(w)

kdxy = 0.660 #Kd(v)
kdw = 0.5 # Kd(w)

kixy = 0 # Ki (v not tuned yet) 
kiw = 0 # Ki (w not tuned yet)

pre_err_x = 0 # previous error
pre_err_y = 0
pre_err_t = 0

int_err_x = 0 # integral error
int_err_y = 0
int_err_t = 0

############################################################################
###################### Tolerance ###########################################
linear_tol = 8  # Not in pixles. (errx^2 + errY^2)^1/2 < linear_tol
angular_tol = 0.07
############################################################################

xListFinal = [] # List to store x co-ordinates of the waypoints.
yListFinal = [] # List to store y co-ordinates of the waypoints.

ip = "192.168.43.13" # Local ip of the system.

############################ Nodes ################################

contourPub = rospy.Publisher('/contours', String, queue_size=10)
cData = String()

penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)
penData = Int32()

taskStatusPub = rospy.Publisher('/taskStatus', Int32, queue_size=10)
taskStatus = Int32()

##################### FUNCTION DEFINITIONS #######################

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	print("clean")
	# msg = f"{0} {0} {0}"
	# conn.send(str.encode(msg))
	# s.close()

def limit_v(v_1, v_2, v_3, x):
	# This function is used to limit the each wheel velocity.
	while abs(v_1) > max_v or abs(v_2) > max_v or abs(v_3) > max_v:
		v_1 = v_1/x
		v_2 = v_2/x
		v_3 = v_3/x
	return int(v_1), int(v_2), int(v_3)  

def imgMode(path, skiping_width):
	xList , yList , xListFinal , yListFinal = [] , [] , [] , []
	xp, yp = [], [] # Complete list of the contours.
	hr_counter = 0
	img = cv2.imread(path) # Reading the image
	img = cv2.resize(img, (500, 500)) # Resizing it to 500x500 
	imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Converting it to greyScale
	imgray = cv2.bitwise_not(imgray) # Inverting the pixels
	ret, thresh = cv2.threshold(imgray, 127, 255, cv2.THRESH_BINARY)
	cnt, hr = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
	for i in cnt:
			xList.clear()
			yList.clear()
			if hr[0][hr_counter][-1] == -1:
				# Store only those contours who are'nt child of any other contour.
				for j in i:
					xList.append(int(j[0][0])) # appending in temporary list.
					yList.append(int(j[0][1]))
				# no of pixles required to skip
				skip = int(skiping_width*len(xList)/1517) # 1517 is the no of complete contours in snapchat image.
				if skip == 0:
					skip = 2
				xListFinal.append(xList[0::skip].copy()) # Reduced List
				yListFinal.append(yList[0::skip].copy())
				# append complete contour list.
				xp.append(xList.copy())
				yp.append(yList.copy())

				if len(xList)%skip !=0:
					# If last waypoint is missing the append it to the list.
					xListFinal[-1].append(xList[-1])
					yListFinal[-1].append(yList[-1])
                # cv2.drawContours(img, i, -1, (0, 255, 0), 3) # to check only outer cnts are appended
			hr_counter=hr_counter+1
	# sort contours based on there lengths.
	xListFinal.sort(key = len)
	yListFinal.sort(key = len)
	return xListFinal, yListFinal, xp, yp # reduced x waypoints, reduced y waypoints, complete x waypoints, complete y waypoints.

def func_mode():
	x_goals, y_goals, theta_goals = [], [], []
	x_list, y_list, theta_list = [], [], []
	for i in range(0,105,3):
		new_value = ( (i - 0) / (102 - 0) ) * (2*3.14  - 0) 
		t=new_value
		x = (400*math.cos(t))/2+250
		x_goals.append(x)
		y = (200*math.sin(2*t))/2+250
		y_goals.append(y)
		theta = (PI/4)*math.sin(t) 
		theta_goals.append(theta)
	x_list.append(x_goals.copy())
	y_list.append(y_goals.copy())
	theta_list.append(theta_goals.copy())
	return x_list, y_list, theta_list # reduced x waypoints, reduced y waypoints, corresponding theta values.
 
def aruco_feedback_Cb(msg):
	# Used to receive odom data
	global hola_x, hola_y, hola_theta
	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta

def endSignalCb(signal):
	# end the task if end signal is received
	if signal==1:
		cleanup()
        
def inverse_kinematics(vel_x, vel_y, vel_z):
	# Implementation of IK to calculate velocity from global frame to body frame.
	v_1 = (-vel_x/2.0 + math.sqrt(3)*vel_y/2.0 - vel_z)
	v_2 = -vel_x/2.0 - math.sqrt(3)*vel_y/2.0 - vel_z
	v_3 = vel_x - vel_z
	# Limiting each wheel velocity
	v_1, v_2, v_3 = limit_v(v_1, v_2, v_3, x)
	return v_1, v_2, v_3

def GTG(currentPos, desiredPos):
	global Kxy, Kw, kdxy, kdw, pre_err_x, pre_err_y, pre_err_t, int_err_x, int_err_y, int_err_t
	# Finding the error in global frame
	err_x = desiredPos[0] - currentPos[0]
	err_y = currentPos[1] - desiredPos[1]
	error_t = desiredPos[2] - currentPos[2]
    # Implementing the PD controller
	derivative_x = (err_x - pre_err_x)
	derivative_y = (err_y - pre_err_y)
	derivative_t = (error_t - pre_err_t)

	int_err_x = (int_err_x + err_x)
	int_err_y = (int_err_y + err_y)
	int_err_t = (int_err_t + err_y)

	gain_x = Kxy*err_x + kdxy*derivative_x  + kixy*int_err_x
	gain_y = Kxy*err_y + kdxy*derivative_y + kixy*int_err_y

	pre_err_x = err_x
	pre_err_y = err_y
	pre_err_t = error_t
	# Velocities in robot body frame
	if math.sqrt(err_x*err_x + err_y*err_y) > linear_tol or abs(error_t) > angular_tol:
		vel_x  = v*(gain_x*math.cos(hola_theta) + gain_y*math.sin(hola_theta))
		vel_y  = v*(-gain_x*math.sin(hola_theta) + gain_y*math.cos(hola_theta))
		vel_z  = Kw*w*error_t + kixy*w*int_err_t +kdw*w*derivative_t
		v_1, v_2, v_3 = inverse_kinematics(vel_x, vel_y, vel_z)	
		return v_1, v_2, v_3, False
	else :
		# If reached to the location
		pre_err_x = 0
		pre_err_y = 0
		pre_err_t = 0

		int_err_x = 0
		int_err_y = 0
		int_err_t = 0
		return 0, 0, 0, True
	
def main():

	rospy.init_node('controller_node')

	signal.signal(signal.SIGINT, signal_handler)

	rospy.Subscriber('detected_aruco',aruco_data,aruco_feedback_Cb)
	rospy.Subscriber('endSignal',Int32,endSignalCb)

	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((ip, 8002))
		s.listen()
		conn, addr = s.accept()

	rate = rospy.Rate(100)
	if mode == "Image":
		xListFinal, yListFinal, xp, yp = imgMode(image, skiping_width)
		# Publishing cntrs
		cData.data = str([xp,yp])
		contourPub.publish(cData)
	elif mode == "Function":
		xListFinal, yListFinal, thetaListFinal = func_mode()
	else:
		print("Specified mode is left to implement....")
		sys.exit()

	next_goal = 0 
	next_cnt = 0

	x_d=xListFinal[next_cnt][next_goal]
	y_d=yListFinal[next_cnt][next_goal] 
	if mode=="Image":
		t_d = 0
	else:

		t_d=thetaListFinal[next_cnt][next_goal]

	# Starting the task
	taskStatus.data = 0
	taskStatusPub.publish(taskStatus)
	print("task started.....")

	# Initially keeping the pen up
	penData.data = 0
	print("penUP")

	# To calculate the runtime.
	starttime = time.time()
	
	while not rospy.is_shutdown():
		# Publishing pen data
		penPub.publish(penData)

		v_1, v_2, v_3, reached = GTG([hola_x, hola_y, hola_theta], [x_d, y_d, t_d])
		msg = f"{-v_3} {-v_1} {-v_2} {penData.data}"
		conn.send(str.encode(msg))
		data = conn.recv(1024) # Two way hand shake from esp.

		if reached:		
			# Loading next goal pose
			if xListFinal[next_cnt][0] == xListFinal[next_cnt][next_goal]:
				# If next waypoint is the first waypoint of the contour then pen down.
				penData.data = 1 # Pen down
				print("pen down")
			elif xListFinal[next_cnt][-1] == xListFinal[next_cnt][next_goal]:
				# If traversed waypoint was the last wp of the contour, then penup
				if penData.data == 1:
					penData.data = 0 # Pen up	
					print("pen up")

			if next_goal == -1:
				next_goal = 0
				next_cnt = next_cnt+1
				if next_cnt > len(xListFinal)-1:
					# If all the waypoints are traversed then end the task.
					endtime = time.time()
					TakenTime = endtime - starttime
					print(f"Image Drawn..{TakenTime}")
					taskStatus.data = 1
					taskStatusPub.publish(taskStatus)
					print("task end")
					break
			else:
				next_goal = next_goal+1 
				if next_goal > len(xListFinal[next_cnt])-1:
					next_goal = -1

			x_d = xListFinal[next_cnt][next_goal]
			y_d = yListFinal[next_cnt][next_goal]
			if mode=="Image":
				t_d = 0
			else:

				thetaListFinal[next_cnt][next_goal]
			
		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

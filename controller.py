#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used in implementation of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:	   [HB_1132]
# Author List: [Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar]
# Filename:	   controller.py
# Functions:   [signal_handler, cleanup, limit_v, aruco_feedback_Cb, inverse_kinematics, GTG, main]
# Nodes:	   Publisher nodes--> '/contours', '/penStatus', '/taskStatus' Subscriber nodes --> 'detected_aruco'
################### IMPORT MODULES #######################

import rospy
import signal							# To handle Signals by OS/user
import sys							    # To handle Signals by OS/user
from std_msgs.msg import String, Int32  # Importing the data types that are used in communication with eval script
import math	                            # For mathematical operations (ex. finding euclidean distance)
import socket						    # For wireless communication b/w base station and hola bot.
import time
import signal		
import sys	
import cv2							    # For extracting contours from image.
import numpy as np                      
from tf.transformations import euler_from_quaternion	# Convert angles
from cv_basics.msg import aruco_data

################## GLOBAL VARIABLES ######################

PI = 3.14

hola_x = 0 # stores x coordinate of hola bot w.r.t arena.
hola_y = 0 # stores y coordinate of hola bot w.r.t arena.
hola_theta = 0 # stores orientation of hola bot w.r.t Y axis.

v = 5 # Linear velocity of Hola bot.
w = 10 # Angular velocity of Hola bot.

x = 1.1 # Dividing constant for prevrnting the each wheel velocity from exceeding the maximum velocity
max_v = 300 # maximum velocity of each wheel

skiping_width = 5 # Number of pexels skip from the contour list in image mode.

Kxy = 2 # Kp value for linear velocity
Kw =  10 # Kp value for angular velocity
kdxy = 0.750 # Kd value for linear velocity
kdw = 0.5 # Kd value for angular velocity

pre_err_x = 0 # Previously calculated error in x axis
pre_err_y = 0 # Previously calculated error in y axis
pre_err_t = 0 # Previously calculated error in theta

linear_tol = 5 # Linear tolerance (not in pixles)
angular_tol = 0.05 # Angular tolerance.

xListFinal = [] # Reduced X list of waypoints.
yListFinal = [] # Reduced X list of waypoints.

t=0
x_cart_eq = 60*math.ceil((4*PI-t)/PI)*math.sin(2*t)*math.cos(t+PI/2)
y_cart_eq = -60*math.ceil((4*PI-t)/PI)*math.sin(2*t)*math.sin(t+PI/2)

ip = "192.168.43.13"  # Ip of the base station.

mode = "Image" # "Image" for image mode, "function" for function mode
############################ Nodes ################################

contourPub = rospy.Publisher('/contours', String, queue_size=10)
cData = String()

penPub = rospy.Publisher('/penStatus', Int32, queue_size=10)
penData = Int32()

taskStatusPub = rospy.Publisher('/taskStatus', Int32, queue_size=10)
taskStatus = Int32()

##################### FUNCTION DEFINITIONS #######################
"""
* Function name: signal_handler
* Input: sig -> 
         frame -> 
* Output: calls cleanup function before termination of the script.
* Logic: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 
* Example Call: signal.signal(signal.SIGINT, signal_handler)
""" 
def signal_handler(sig, frame):
	print('Clean-up !')
	cleanup()
	sys.exit(0)

"""
* Function name: cleanup
* Input: None
* Output: 
* Logic:
* Example Call:cleanup()
""" 
def cleanup():
	print("clean")
	# msg = f"{0} {0} {0}"
	# conn.send(str.encode(msg))
	# s.close()

"""
* Function name: limit_v
* Input:  v_1 -> velocity of front wheel
*         v_2 -> velocity of right wheel
*         v_3 -> velocity of left wheel
*         x -> Dividing constant
* Output: returns velocity under limit
* Logic: checks if any velocity is greater than the maximum velocity then divides all the velocity with dividing constant 'x'
*         if all the velocity becomes under the limit it returns valid velocity (keeps the ratio of each wheel velocity maintained) 
* Example Call: v_1, v_2, v_3 = limit_v(v_1, v_2, v_3, x)
""" 
def limit_v(v_1, v_2, v_3, x):
	while abs(v_1) > max_v or abs(v_2) > max_v or abs(v_3) > max_v:
	# If any wheel velocity is more than the maximum wheel velocity then keep dividing.
		v_1 = v_1/x
		v_2 = v_2/x
		v_3 = v_3/x
	return int(v_1), int(v_2), int(v_3)  

"""
* Function name: imgMode
* Input: path (absolute image path), skiping_width 
* Output: xListFinal (reduced x list of contours), yListFinal (reduced y list of contours)
*         pen_status (pen status for each contour), xp (complete x contour list), yp (complete y contour list)
* Logic: This function extracts the contours from the passed image as argument and returns the reduced, complete
*        as well as the pen status for the each contour.
*        After extracting any contour from image it appends the x and y coordinates in respective full contour list
*        it also checks if last coordinate of any contour is being appended then append the '0' (pen up) in the pen status 
*        else append '1' (keeps the pen down) in penstatus.
* Example Call: xListFinal, yListFinal, pen_status, xp, yp = imgMode(image, skiping_width)
""" 
def imgMode(path, skiping_width):
	xList , yList , xListFinal , yListFinal, pen_status = [] , [] , [] , [], []
	xp, yp = [], [] # temporary list to store only single contour.
	temp_pen_status = [] # stores pen data for each waypoint.

	img = cv2.imread(path) # reading the image
	img = cv2.resize(img, (500, 500)) # resizing it to arena size.
	imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # converting it to grayscale.
	ret, thresh = cv2.threshold(imgray, 100, 255, cv2.THRESH_BINARY) # applying threshold (If the pixel value is smaller than the threshold, it is set to 0, otherwise it is set to a maximum value.)
	cnt, hr = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # Finding contours.
	# Segregating x and y coordinates from contours and appending it 
	# appending it respective lists for full contour list.
	# appending every contour after skiping_width inrespective reduced contour list
	# also checking if the last contour is being appended or not (for max accuracy and completeness of the closed curve)
	# maintaining a binary nested list same as reduced waypoint list for the pen status (if last waypoint of any curve is being appended, append 0 in the
	# list else append 1)
	for i in cnt:
			xList.clear()
			yList.clear()
			temp_pen_status.clear()
			for j in i:
				xList.append(int(j[0][0]))
				yList.append(int(j[0][1]))
				temp_pen_status.append(1)
			xListFinal.append(xList[0::skiping_width].copy())
			yListFinal.append(yList[0::skiping_width].copy())
			pen_status.append(temp_pen_status[0::skiping_width].copy())
			xp.append(xList.copy())
			yp.append(yList.copy())
			if len(xList)%skiping_width !=0:
				xListFinal[-1].append(xList[-1])
				yListFinal[-1].append(yList[-1])
				pen_status[-1].append(1)
			pen_status[-1][-1]=0
	# Removing the boundary contour
	xListFinal.pop(0)
	yListFinal.pop(0)
	pen_status.pop(0)
	xp.pop(0)
	yp.pop(0)
	# Sorting the contour list according to length of the contour list. (Draw smallest first)
	xListFinal.sort(key = len)
	yListFinal.sort(key = len)
	pen_status.sort(key=len)
	return xListFinal, yListFinal,pen_status, xp, yp

max_t = 720

"""
* Function name: func_Mode
* Input: path max_t, skiping_width 
* Output: xListFinal (reduced x list of x coordinates), yListFinal (reduced y list of y coordinates)
*         thetaListFinal (reduced x list of theta coordinates(orientation of bot))
* Logic: This function first maps all radian values of goal position i.e. [0,4*PI] in degree
*        then the values in radians are passed to respective x and y parametric equations.
*        For the orientation part we have divided it in two parts intial [0,2*PI] i.e. Big curve and then [2*PI,4*PI] 
*        Finally all the goal points are appended in x ,y & theta list 
*        Then all list are appended in a nested list for a generalized solution
* Example Call: xListFinal, yListFinal, thetaListFinal = func_mode(720, skiping_width)
""" 
def func_mode(max_t, skipping_width):
	temp_x_goals=[] # To store X waypoints.
	temp_y_goals=[] # To store Y waypoints.
	temp_theta_goals=[] # To store respective theta values.

	x_goals = [] # To store X waypoints.
	y_goals = [] # To store Y waypoints.
	theta_goals=[] # To store respective theta values.
	#mapping values in degrees
	for t in range(0,max_t):   
		#converting in radians from degrees                                                           
		t=t* PI / 180		                                                              
		x=60*math.ceil((4*PI-t)/PI)*math.sin(2*t)*math.cos(t+PI/2)+250                    
		temp_x_goals.append(x)
		y=-60*math.ceil((4*PI-t)/PI)*math.sin(2*t)*math.sin(t+PI/2)+250
		temp_y_goals.append(y)
		# theta_t = ( (t - 0) / (max_t - 0) ) * (PI/4 - (-3*PI/4)) 
		#calculating theta values for bigger curve
		if(t<2*PI):                                                                    
			theta = (2*t)
			# Making the theta values under range in for hola bot.
			if theta>PI:
				theta =(2*PI -theta)
			if theta < -PI:
				theta = -(2*PI + theta)
		#calculating theta values for smaller curve
		elif(t>2*PI):                                                                   
			t_smaller=t
			t_smaller=t_smaller-2*PI
			theta = (2*t_smaller)
			# Making the theta values under range in for hola bot.
			if theta>PI:
				theta =(2*PI -theta)
			if theta < -PI:
				theta = -(2*PI + theta)
		temp_theta_goals.append(theta)
	# Skipping some waypoints for fatser movements.
	x_goals.append(temp_x_goals[0::skipping_width].copy())
	y_goals.append(temp_y_goals[0::skipping_width].copy())
	theta_goals.append(temp_theta_goals[0::skipping_width].copy())

	return x_goals, y_goals, theta_goals

"""
* Function name: aruco_feedback_Cb
* Input: msg
* Output: Update the current position and orintation of the hola bot.
* Logic: Automatically called when there is any feedback from overhead camera.
*        Receives odom data and update the variables.
* Example Call: rospy.Subscriber('detected_aruco',aruco_data,aruco_feedback_Cb)
""" 
def aruco_feedback_Cb(msg):
	# msg contains x-> x coordinate of hola bot, y-> y coordinate of hola bot, theta-> orientation of the bot.
	global hola_x, hola_y, hola_theta
	hola_x = msg.x 
	hola_y = msg.y
	hola_theta = msg.theta

"""
* Function name: inverse_kinematics
* Input: vel_x -> velocity of bot in x direction w.r.t. global frame
*        vel_y -> velocity of bot in y direction w.r.t. global frame
*	     vel_z -> velocity of bot in z direction w.r.t. global frame
* Output: v_1 ->  velocity of front wheel
*         v_2 ->  velocity of right wheel
*	      v_3 ->  velocity of left wheel
* Logic: This function is called for calculating each wheel velocity for any goal location.
*        converts global frame velocity to body frame velocity.
* Example Call:v_1, v_2, v_3 = inverse_kinematics(vel_x, vel_y, vel_z)	
"""      
def inverse_kinematics(vel_x, vel_y, vel_z):
	# vel_x -> velocity in x axis wrt global frame
	# vel_y -> velocity in y axis wrt global frame
	# vel_z -> velocity in z axis wrt global frame
	# finding each wheel velocity. (each wheel at 60 degree)
	v_1 = (-vel_x/2.0 + math.sqrt(3)*vel_y/2.0 - vel_z)
	v_2 = -vel_x/2.0 - math.sqrt(3)*vel_y/2.0 - vel_z
	v_3 = vel_x - vel_z
	v_1, v_2, v_3 = limit_v(v_1, v_2, v_3, x)
	return v_1, v_2, v_3

"""
* Function name: GTG (go-to-goal)
* Input: currentPos, desiredPos (index 0, 1, 2 => x, y, theta)
* Output: Returns each wheel velocity for perticular goal location also returns true if the bot reached to the goal pose
* Logic: It is called when controller wants move bot from one location to another.
*        Calculates the errors in linear and angular positions then calculates the velocity in global frame after calculating gain
*        for each axis, then converts it to robot body frame velocity by calling inverse_kinematics.
* Example Call: v_1, v_2, v_3, reached = GTG([hola_x, hola_y, hola_theta], [x_d, y_d, t_d])
""" 
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

	# gain for linear velocity
	gain_x = Kxy*err_x + kdxy*derivative_x 
	gain_y = Kxy*err_y + kdxy*derivative_y

	# Velocities in robot body frame
	if math.sqrt(err_x*err_x + err_y*err_y) > linear_tol or abs(error_t) > angular_tol: 
		# If the error is more than the tolerance, keep correcting it.
		vel_x  = v*(gain_x*math.cos(hola_theta) + gain_y*math.sin(hola_theta))
		vel_y  = v*(-gain_x*math.sin(hola_theta) + gain_y*math.cos(hola_theta))
		vel_z  = Kw*w*error_t +kdw*w*derivative_t
		v_1, v_2, v_3 = inverse_kinematics(vel_x, vel_y, vel_z)	
		return v_1, v_2, v_3, False
	else :
		# If error is less than tolerance, reached to the goal
		# stop the bot
		# reset the parameters for D controller
		pre_err_x = 0
		pre_err_y = 0
		pre_err_t = 0
		return 0, 0, 0, True
	
"""
* Function name: main
* Input:  None
* Output: None
* Logic: Takes feedback from overhead camera by subscribing to the 'detected_aruco' topic and controlles the bot accordingly
* Example Call:main()
"""  
def main():

	rospy.init_node('controller_node')
	signal.signal(signal.SIGINT, signal_handler)
	# Subscribe to the 'detected_aruco' topic for feedback
	rospy.Subscriber('detected_aruco',aruco_data,aruco_feedback_Cb)

	# Wireless connection with hola bot
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((ip, 8002)) # binds address (hostname, port number pair) to socket
		s.listen() #sets up and start TCP listener.
		conn, addr = s.accept() # Waiting for client (Hola bot) to connect
	rate = rospy.Rate(100)

	# Absolute path of the image
	image = "/home/shashwat/ros_ws/src/hola_bot/scripts/org.png"
	if mode == "Image":
		xListFinal, yListFinal, xp, penS, yp = imgMode(image, skiping_width)
		# Publishing cntrs
		cData.data = str([xp,yp])
		contourPub.publish(cData)
	elif mode == "Function":
		xListFinal, yListFinal, thetaListFinal = func_mode(max_t, skiping_width)
	else:
		print("Specified mode is left to implement....")
		sys.exit()


	next_goal = 0 # Next waypoint
	next_cnt = 0 # Next contour list
    
	x_d=xListFinal[next_cnt][next_goal] # desired X (loading x waypoint)
	y_d=yListFinal[next_cnt][next_goal] # desired Y (loading y waypoint)

	if mode=="Image":
		t_d = 0
	else:

		t_d=thetaListFinal[next_cnt][next_goal]

	# Starting the task
	# Publishing the task status for evaluation
	taskStatus.data = 0
	taskStatusPub.publish(taskStatus)
	
	print("task started.....")

	# Initially keeping the pen up
	penData.data = 0
	pre_penData = 0 # To store the previous pen status
	print("penUP")
	penPub.publish(penData) # Publishing the pen status

	# To calculate the Runtime (Only for debugging)
	starttime = time.time()
	while not rospy.is_shutdown():
		# Calling go-to-goal to calculate each wheel velocity for current goal.
		v_1, v_2, v_3, reached = GTG([hola_x, hola_y, hola_theta], [x_d, y_d, t_d])

		# After calculating sending it to hola bot (Orientation of the motors is different than the expected)
		msg = f"{-v_3} {-v_1} {-v_2}\n"
		conn.send(str.encode(msg))
		conn.recv(1024) # Two way hand shake from esp.

		if reached:	
			# If hola bot reaches to the goal location
			if mode == "Image":
				#Checks the pen status
				penData.data=penS[next_cnt][next_goal]
				penPub.publish(penData) # Publishes the pen status 
				#Also if there is change in pen status then send it to the hola bot.
				if pre_penData != penData.data:
					msg = f"p{penData.data}\n"
					conn.send(str.encode(msg))
					conn.recv(1024)
					pre_penData = penData.data # Update the previous pen status
			next_goal = next_goal+1 
			if next_goal > len(xListFinal[next_cnt])-1:
				next_goal = 0
				next_cnt = next_cnt+1
				if next_cnt > len(xListFinal)-1:
					# If all the waypoints are traversed then end the task.
					endtime = time.time() # 
					TakenTime = endtime - starttime # Calculating the runtime.
					print(f"Image Drawn..{TakenTime}")
					if mode=="Image":
						taskStatus.data = 1
						taskStatusPub.publish(taskStatus)
						msg = f"p{penData.data}\n"
						conn.send(str.encode(msg))
						conn.recv(1024)
						print("task end")
					break
			x_d = xListFinal[next_cnt][next_goal]
			y_d = yListFinal[next_cnt][next_goal]
			if mode=="Image":
				t_d = 0
				penData.data = penS[next_cnt][next_goal]
			else:
				thetaListFinal[next_cnt][next_goal]
		rate.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

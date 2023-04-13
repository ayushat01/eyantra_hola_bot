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

# Team ID:		[HB_1132]
# Author List:		[Shashwat Patel, Ayush Tripathi, Arunesh Sagar, Ankit Ahirwar]
# Filename: aruco.py
# Functions: [localization, detect_aruco, callback, main]
# Nodes:	Publishing node -> 'detected_aruco'(to publish odom data) Subscribing node -> 'usb_cam/image_rect'(to get overhead camera feed)
# Global Variables: width, height, tl_id, tr_id, bl_id, br_id, warp_id, hola_id, hola_trajectory_pts, aruco_publisher, aruco_msg
######################## IMPORT MODULES ##########################

import numpy as np
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2	# OpenCV Library
import cv2.aruco as aruco
import math	
from cv_basics.msg import aruco_data # Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', aruco_data)
aruco_msg = aruco_data()

# size of feedback window (used in warp perspective transform)
width = 500
height = 500
# aruco id's pasted on arena corners
tl_id = 8 # top left aruco id
tr_id = 10 # top right aruco id
bl_id = 4 # bottom left aruco id
br_id = 12 # bottom right aruco id

warp_id = {} # to store id, bounding boxes pairs for warp perspective
hola_id = 15 # hola bot aruco id
hola_trajectory_pts = [] # contains all the coordinates travelled by hola bot to plot trajectory (only for debugging purpose)

##################### FUNCTION DEFINITIONS #######################
"""
* Function name: localization.
* Input: bboxs -> numpy array containing corner coordinates of aruco which is pasted on hola bot.
* Output: x_centerPixel -> x coordinate of hola bot w.r.t. arena.
*         y_centerPixel -> y coordinate of hola bot w.r.t. arena.
*         theta -> orientation of hola bot w.r.t. the y axis.  
* Logic: This function is used in finding the position and orientation of the hola bot wrt global frame.
*		 after getting the 4 corners of aruco (hola_id) as input we calculated the centroid of the square
*		 to get the x and y coordinates of the hola bot. And for finding the orientation we calculated the
*		 slope of any verticle edge of the square.
* Example Call: aruco_msg.x, aruco_msg.y, aruco_msg.theta = localization(hb[hola_id]).
"""
def localization(bboxs):
	global hola_trajectory_pts
	Tl = [bboxs[0][0] , bboxs[0][1] ] # top left left corner coordinates 
	Tr = [bboxs[1][0] , bboxs[1][1] ] # top right corner coordinates
	Br = [bboxs[2][0] , bboxs[2][1] ] # bottom right corner coordinates
	Bl = [bboxs[3][0] , bboxs[3][1] ] # bottom left corner coordinates
	# calculating centeroid
	x_sum = Tl[0]+ Tr[0]+ Br[0]+ Bl[0]
	y_sum = Tl[1]+ Tr[1]+ Br[1]+ Bl[1]
	x_centerPixel = x_sum*.25 # x coordinate of the hola bot w.r.t arena.
	y_centerPixel = y_sum*.25 # y coordinate of the hola bot w.r.t arena.
	# calculating slope of verticle edge of the aruco to get the orientation.
	x = Br[0]-Bl[0]
	y = Tr[1]-Tl[1]
	theta = math.atan2(-y, x) # orientation of the hola bot w.r.t arena.
	# appending the current location of the hola bot in trajectory list (only for debugging purpose)
	hola_trajectory_pts.append((x_centerPixel, y_centerPixel))
	return x_centerPixel, y_centerPixel, theta
"""
* Function name: detect_aruco.
* Input: current_frame -> camera frame(arena only) found after applying warp prespective.
* Output: Dictionary containing id of aruco as key and corner coordinates (bounding boxes) of aruco as elements.
* Logic: This function is used in detecting hola bot on the arena.
*        Detection of aruco using cv2.aruco library and its function.
* Example Call:  hb = detect_aruco(imgOut)
"""
def detect_aruco(current_frame):
	hola_aruco = {} # to store id:bboxes of the aruco's on the arena 
	arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250) # grabing the dictionary of ArUco markers we’re using.
	arucoParam = aruco.DetectorParameters_create() # defining the ArUco detection parameters
	bboxs, ids, rejected = aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParam) # performing ArUco marker detection
	aruco.drawDetectedMarkers(current_frame, bboxs) # indicating detected marker
	if ids is not None:
		for i in range(len(ids)):
			ls = bboxs[i][0]
			ls = ls.tolist()
			key = ids[i][0]
			hola_aruco[key] = ls # contains id-corners pairs of each aruco
	return hola_aruco
"""
* Function name: callback.
* Input: data -> Contains data which is being published on '/usb_cam/image_rect' topic(feedback from overhead camera).
* Output: Displays the feedback window from overhead camera after applying warp perspective.
* Logic: This function receives overhead camera feed then apply warp perspective to extract arena only.
*        It calls detect_aruco and localization function to get the odom data then publishes the same on detect_aruco topic.
* Example Call: rospy.Subscriber('/usb_cam/image_rect', Image, callback).
"""
def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	current_frame = br.imgmsg_to_cv2(data, "mono8") # Receiving raw image in a "grayscale" format
	# applying warp perspective
	arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParam = aruco.DetectorParameters_create()
	bboxs, ids, rejected = aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParam)
	for i in range(len(ids)):
		ls = bboxs[i][0]
		ls = ls.tolist()
		key = ids[i][0]
		warp_id[key] = ls # dictionary contains id-corners pairs of each arena corner aruco
	# selecting all the above four aruco corners points in an array
	pts1 = np.float32([warp_id[tl_id][0], warp_id[bl_id][3], warp_id[tr_id][1], warp_id[br_id][2]])
	# destination image
	pts2 = np.float32([[0, 0], [0, 500], [500, 0], [500, 500]])
	matrix = cv2.getPerspectiveTransform(pts1, pts2) # will return the matrix which will be fed into warpPerspective function to get the warped image.
	imgOut = cv2.warpPerspective(current_frame, matrix, (width, height), flags=cv2.INTER_LINEAR) # imgOut will contains 500x500 arena only after applying perspective transform.
	# detecting the hola bot on the arena 
	hb = detect_aruco(imgOut)
	if len(hb)!=0 and hola_id in hb:
		# printing the hola id on the aruco pasted on hola bot.
		cv2.putText(imgOut, f"id={str(hola_id)}", (int(hb[hola_id][1][0]), int(hb[hola_id][1][1])), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (255, 0, 0), 2, cv2.LINE_AA)
		# finding the position and orientation of the hola bot.
		aruco_msg.x, aruco_msg.y, aruco_msg.theta = localization(hb[15])
		# publishing the odom data on the detect_aruco topic.
		aruco_publisher.publish(aruco_msg)

	# uncomment if you want to draw trajectory on the feedback window. (for debugging purpose only)
	# for i in hola_trajectory_pts:
	# 	cx = int(i[0])
	# 	cy = int(i[1])
	# 	cv2.circle(imgOut, (cx, cy), 2, (0, 0, 0), -1)

	# printing the hola data on the feedback window
	cv2.putText(imgOut, f"{str(aruco_msg.x)}, {str(aruco_msg.y)}", (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 0, 0), 2, cv2.LINE_AA)
	cv2.putText(imgOut, f"{str(round(aruco_msg.theta, 3))}", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 0, 0), 2, cv2.LINE_AA)
	cv2.imshow("warp", imgOut)
	cv2.waitKey(3)
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('/usb_cam/image_rect', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()

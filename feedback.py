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

# Team ID:		HB_1132
# Author List:	Shashwat Patel , Ayush Tripathi , Arunesh Kumar Sagar , Ankit Ahirwar
# Filename:		feedback.py
# Functions:
#			callback() , main()
# Nodes:	detected_aruco (Publisher) , overhead_cam/image_raw (Subscriber)


######################## IMPORT MODULES ##########################

import numpy as np				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import cv2.aruco as aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation


############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)


	arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParam = aruco.DetectorParameters_create()
	bboxs, ids, rejected = aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParam)
	aruco.drawDetectedMarkers(current_frame, bboxs)

	Tl = [bboxs[0][0][0][0], bboxs[0][0][0][1]]
	Tr = [bboxs[0][0][1][0], bboxs[0][0][1][1]]
	Br = [bboxs[0][0][2][0], bboxs[0][0][2][1]]
	Bl = [bboxs[0][0][3][0], bboxs[0][0][3][1]]

	x_sum = bboxs[0][0][0][0]+ bboxs[0][0][1][0]+ bboxs[0][0][2][0]+ bboxs[0][0][3][0]
	y_sum = bboxs[0][0][0][1]+ bboxs[0][0][1][1]+ bboxs[0][0][2][1]+ bboxs[0][0][3][1]
		
	x_centerPixel = x_sum*.25
	y_centerPixel = y_sum*.25
	x = Br[0]-Bl[0]
	y = Tr[1]-Tl[1]
	theta = math.atan2(-y, x)
	aruco_msg.x = x_centerPixel
	aruco_msg.y = y_centerPixel
	aruco_msg.theta = theta
	aruco_publisher.publish(aruco_msg)
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()

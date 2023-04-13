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
# Filename:		feedback.py
# Functions:
#			[localization, detect_aruco, callback, main]
# Nodes:		Add your publishing and subscribing node


######################## IMPORT MODULES ##########################

import numpy as np				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
import cv2.aruco as aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation
from cv_basics.msg import aruco_data


width = 500
height = 500
ar = {}
tl_id = 8
tr_id = 10
bl_id = 4
br_id = 12
hola_id = 15

pts = []
############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', aruco_data)
aruco_msg = aruco_data()
##################### FUNCTION DEFINITIONS #######################

def localization(bboxs):
	global pts

	Tl = [bboxs[0][0] , bboxs[0][1] ]
	Tr = [bboxs[1][0] , bboxs[1][1] ]
	Br = [bboxs[2][0] , bboxs[2][1] ]
	Bl = [bboxs[3][0] , bboxs[3][1] ]

	x_sum = Tl[0]+ Tr[0]+ Br[0]+ Bl[0]
	y_sum = Tl[1]+ Tr[1]+ Br[1]+ Bl[1]
		
	x_centerPixel = x_sum*.25
	y_centerPixel = y_sum*.25
	x = Br[0]-Bl[0]
	y = Tr[1]-Tl[1]
	theta = math.atan2(-y, x)
	pts.append((x_centerPixel, y_centerPixel))
	return x_centerPixel, y_centerPixel, theta

def detect_aruco(current_frame):
	ar = {}
	arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParam = aruco.DetectorParameters_create()
	bboxs, ids, rejected = aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParam)
	aruco.drawDetectedMarkers(current_frame, bboxs)
	if ids is not None:
		for i in range(len(ids)):
			ls = bboxs[i][0]
			ls = ls.tolist()
			key = ids[i][0]
			ar[key] = ls # dictionary contains id-corners pairs of each aruco
	return ar


def callback(data):
	global pts
	# Bridge is Used to Convert ROS Image message to OpenCV image
	br = CvBridge()
	current_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
	# current_frame = cv2.resize(current_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

	arucoDict = aruco.Dictionary_get(aruco.DICT_4X4_250)
	arucoParam = aruco.DetectorParameters_create()
	bboxs, ids, rejected = aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParam)
	# aruco.drawDetectedMarkers(current_frame, bboxs)

	for i in range(len(ids)):
		ls = bboxs[i][0]
		ls = ls.tolist()
		key = ids[i][0]
		ar[key] = ls # dictionary contains id-corners pairs of each aruco
	pts1 = np.float32([ar[tl_id][0], ar[bl_id][3], ar[tr_id][1], ar[br_id][2]])
	pts2 = np.float32([[0, 0], [0, 500], [500, 0], [500, 500]])
	matrix = cv2.getPerspectiveTransform(pts1, pts2)
	imgOut = cv2.warpPerspective(current_frame, matrix, (width, height), flags=cv2.INTER_LINEAR)
	hb = detect_aruco(imgOut)

	if len(hb)!=0 and hola_id in hb:
		cv2.putText(imgOut, f"id={str(hola_id)}", (int(hb[hola_id][1][0]), int(hb[hola_id][1][1])), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.5, (255, 0, 0), 2, cv2.LINE_AA)
		aruco_msg.x, aruco_msg.y, aruco_msg.theta = localization(hb[15])
		aruco_publisher.publish(aruco_msg)
		# imgOut = cv2.circle(imgOut, (int(aruco_msg.x), int(aruco_msg.y)), 1, (255, 0, 0), 1)
		# print(aruco_msg)

	# Draw Trajectory of the Hola Bot

	# for i in pts:
	# 	cx = int(i[0])
	# 	cy = int(i[1])
	# 	cv2.circle(imgOut, (cx, cy), 2, (0, 0, 0), -1)

	cv2.putText(imgOut, f"{str(aruco_msg.x)}, {str(aruco_msg.y)}", (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 0, 0), 2, cv2.LINE_AA)
	cv2.putText(imgOut, f"{str(round(aruco_msg.theta, 3))}", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (255, 0, 0), 2, cv2.LINE_AA)

	# cv2.imshow("frame", current_frame)
	cv2.imshow("warp", imgOut)
	cv2.waitKey(3)
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('/usb_cam/image_rect', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()

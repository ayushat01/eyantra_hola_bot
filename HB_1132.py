#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			HB#1132
# Author List:		Shashwat Patel,Ankit Ahirwar,Arunesh Kumar Sagar,Ayush Tripathi
# Filename:			task_0.py
# Functions:
# 					[ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
 

####################### IMPORT MODULES #######################
from ast import Break
from re import sub
import sys
import traceback
from enum import Flag
import math
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
##############################################################



def callback(data):
    global step, curr_x, curr_y
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist() 
    if step == 1:
        # draw circle
        circle(data, velocity_publisher, vel_msg)
    elif step == 2:
        # fcaing downward
        rotate(data, velocity_publisher, vel_msg)
    elif step == 3:
        # Moving in a straight line
        straight(data, velocity_publisher, vel_msg)
    else:
        # after completing D, finishing process cleanly 
        rospy.signal_shutdown("Finishing")



def main():
    rospy.init_node("turtle_controller")
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=callback)
    rospy.spin()


################# ADD GLOBAL VARIABLES HERE #################
curr_x = 0 # curr_x stores initial x coordinate
curr_y = 0 # curr_y stores initial y coordinate
final_x = 0 # final_x stores x coordinate from where turtle starts moving in a straight line
final_y = 0 # final_y stores y coordinate from where turtle starts moving in a straight line
step = 1 # whole D is divided into 4 parts (1-Circle, 2-rotation, 3-straight line, 4-termination)
##############################################################


################# ADD UTILITY FUNCTIONS HERE #################
def circle(data, velocity_publisher, vel_msg):
    global step, curr_x, curr_y
    vel_msg.linear.x = 1 # setting linear and angular velocity as 1 (radius)
    vel_msg.angular.z = 1
    if curr_y == 0 and curr_x == 0: # storing initial coordinates
        curr_x = data.x
        curr_y = data.y
    velocity_publisher.publish(vel_msg) # publishing velocity components
    print("My turtlbot is : Moving in circle!!")
    print(round(data.theta,2))
    # checking if the semi-circle is completed, make angular and linear velocity components as 0
    # and making step = 2 so that turtle can enter in rotating mode in next callback
    if pow((pow((data.x-curr_x), 2)+pow((data.y-curr_y), 2)), 1/2) >= 2:
        vel_msg.linear.x=0
        vel_msg.angular.z = 0
        step = 2
        velocity_publisher.publish(vel_msg)

def rotate(data, velocity_publisher, vel_msg):
    global step, final_x, final_y
    # making linear velocity 0 and angular velocity 1
    vel_msg.linear.x=0
    vel_msg.angular.z = 1
    # checking if the turtle is facing down or not, if not then keep publishing angular velocity
    # else capture it's current co-ordinates as final_x, final_y (for straight line movement), 
    # make angular velocity 0 and also change step to 3 so that turtle can enter in straight line
    # movement in next callback
    if (abs(data.theta)*180*7)/22 >= 90:
        velocity_publisher.publish(vel_msg)
        print("My turtlbot is : Rotating!")
        print(round(data.theta,2))
    else:
        final_x = data.x
        final_y = data.y
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        step = 3 

def straight(data, velocity_publisher, vel_msg):
    global step, final_x, final_y
    # calculating distance between (final_x, final_y) and (data.x, data.y)
    dis = pow((pow((data.x-final_x), 2)+pow((data.y-final_y), 2)), 1/2)
    # for straight line movement setting only linear velocity component
    vel_msg.linear.x = 1
    # checking if distance (dis) is less than 2, keep publishing linear velocity
    # if distance becomes more than 2, make linear velocity 0 and step = 4 for termination in next callback
    if dis<=2:
        print("My turtlbot is : Moving Straight!!!")
        print(round(data.theta,2))
        velocity_publisher.publish(vel_msg)
    else:
        vel_msg.linear.x=0
        velocity_publisher.publish(vel_msg)
        print("Done!!")
        step = 4
##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")

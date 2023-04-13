#!/usr/bin/env python3

import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
from math import *
import time
from geometry_msgs.msg import PoseArray

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

hola_x = 0
hola_y = 0
hola_theta = 0

pi = 3.14159265

# x_goals = [0]
# y_goals = [0]
# theta_goals = [0]
 
# x_goals = [1,0,0,0,1]
# y_goals = [0,1,0,0,0]
# theta_goals = [0, 0, 0, 3, -3]
# x_goals = [1,-1,-1,1,0]
# y_goals = [1,1,-1,-1,0]
# theta_goals = [0.785, 2.335, -2.335, -0.785, 0]

x_goals = [0,1,-1,0,0]
y_goals = [1,-1,-1,1,-1]
theta_goals = [0, 0, 0, 0, 0]

def task1_goals_Cb(msg):
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

def odometryCb(msg):
    global hola_x, hola_y, hola_theta
    hola_x = msg.pose.pose.position.x
    hola_y = msg.pose.pose.position.y
    orientaion = msg.pose.pose.orientation
    (roll, pitch, hola_theta) = euler_from_quaternion([orientaion.x, orientaion.y, orientaion.z, orientaion.w])   


def main():
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
    rospy.init_node("controller")
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
    rospy.Subscriber("/odom", Odometry, odometryCb)
    rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

	# Declare a Twist message
    vel = Twist()

    next_goal = 0
    x_d=x_goals[next_goal]
    y_d=y_goals[next_goal] 
    t_d=theta_goals[next_goal]
    

    v = 5
    w = 5
    max_v = 10

    Kx = 0.7
    Ky = 0.7
    Kw =  1.550 
	# For maintaining control loop rate.
    rate = rospy.Rate(100)

	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# and also Kp values for the P Controller
    while not rospy.is_shutdown():

		# Find error (in x, y and theta) in global frame
        err_x = x_d - hola_x
        err_y = y_d - hola_y        
        error_t=t_d-hola_theta
        #implementing the P controller
        gain_x = Kx*err_x
        gain_y = Ky*err_y

        if sqrt(err_x*err_x + err_y*err_y) > 0.01 or abs(error_t) > 0.01:
            # If goal position or the desired orientaion is not achieved, keep publishing velocities.
            vel.linear.x  = v*(gain_x*cos(hola_theta) + gain_y*sin(hola_theta))
            vel.linear.y  = v*(-gain_x*sin(hola_theta) + gain_y*cos(hola_theta))
            vel.angular.z  = Kw*w*error_t
            # Check if the velocities are range or not
            if vel.linear.x >= 0:
                vel.linear.x = min(vel.linear.x, max_v)
            else:
                vel.linear.x = max(vel.linear.x, -max_v)
            if vel.linear.y >= 0:
                vel.linear.y = min(vel.linear.y, max_v)
            else:
                vel.linear.y = max(vel.linear.y, -max_v)
            if vel.angular.z >= 0:
                vel.angular.z = min(vel.angular.z, max_v)
            else:
                vel.angular.z = max(vel.angular.z, -max_v) 
            pub.publish(vel)
        else:
            # If goal position is reached, making all the velocities zero, stablize it for 1 second
            # and changing the current goal to new goal
            next_goal = next_goal+1            
            if next_goal > len(x_goals)-1:
                next_goal = 0
            x_d = x_goals[next_goal]
            y_d = y_goals[next_goal]
            t_d = theta_goals[next_goal]
            vel.linear.x  =0
            vel.linear.y  =0
            vel.angular.z  =0
            pub.publish(vel)
            print(hola_theta)
            rospy.sleep(1) 
        rate.sleep()
		

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
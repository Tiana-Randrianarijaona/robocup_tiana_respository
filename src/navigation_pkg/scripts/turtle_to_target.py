#!/usr/bin/env python3

import rospy  # import rospy
from turtlesim.msg import Pose   # import Pose message 
from geometry_msgs.msg import Twist   # import Twist message
from math import atan2, sqrt  # import the mathematical functions atan2 and sqrt from the python module called math
import sys

target_x = 0   # initialize the x coordinate of the target
target_y = 0   # initialize the y coordinate of the target
x = 0          # initialize the x coordinate of the turtle
y = 0          # initialize the y coordinate of the turtle
yaw = 0        # initialize the orientation of the turtle


def pose_callback(pose):
    global x, y, yaw
    rospy.loginfo("x = %f, y = %f, yawn = %f\n", pose.x, pose.y, pose.theta)
    x = pose.x
    y = pose.y
    yaw = pose.theta

    # printout the x and y position of the turtle1 in the consol (just for debugg)
    print(f"x = {x}, y = {y}")  


def pose_target_callback(pose):
    global target_x
    global target_y
    rospy.loginfo("x = %f, y = %f\n", pose.x, pose.y)
    target_x = pose.x
    target_y = pose.y

    # printout the x and y position of the turtle2 in the consol (just for debugg)
    print(f"x = {target_x}, y = {target_x}")  



def turtle_to_target():
    # Initialise the node
    rospy.init_node('turtle_to_target', anonymous=False)

    # Create a subscriber to the turtle2 pose topic (our target) in order to find its position
    # sub_target = rospy.Subscriber('/turtle2/pose', Pose, pose_target_callback)

    # Create a subscriber to the raw_odom topic    
    sub = rospy.Subscriber('/raw_odom', Pose, pose_callback)

    # Create a publisher to the robot's velocity command
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

    rate = rospy.Rate(10) # 10hz
 
    vel = Twist() # creates a Twist object named vel
    
    while (True):
        K_linear = 0.5

        distance = sqrt((target_x - x)**2 + (target_y - y)**2)     # Computes the Euclidean distance between current pose and the target
	    
        # printout the distance in the consol (just for debugg)	
        rospy.loginfo(distance)

        linear_speed = K_linear * distance  # computes the velocity of the proportional controller

        K_angular = 4.0
        steering_angle = atan2(target_y-y,target_x-x)
        angular_speed = (steering_angle - yaw) * K_angular

        vel.linear.x = linear_speed
        vel.angular.z = angular_speed

	    # publish the value of the velocity for the turtle1
        pub.publish(vel)	

        if (distance <0.01):  # Explain why we do this
            break

        rate.sleep()



if __name__ == '__main__':
    try:


        turtle_to_target()

    except rospy.ROSInterruptException:
        pass

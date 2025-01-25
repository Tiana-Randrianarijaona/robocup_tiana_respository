#! /usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist 

def move_turtle(lin_vel,angle_vel):
    #initiate node
    rospy.init_node('move_robot_linear_node', anonymous = True)


    #create a publisher to talk to Turtlesim
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)

    #Create a Twist message and add lienar x values
    vel = Twist()
    vel.linear.x = 1.0 #Move along the x axis only

    # #Save current time and set publish rate at 10 Hz
    # tStart = rospy.Time.now()
    rate = rospy.Rate(10)

    # #For the next 6 seconds, publish vel move commands to Turtlesim
    # while rospy.Time.now() < tStart + rospy.Duration.from_sec(6):
    #     pub.publish(vel)
    #     rate.sleep()

    while not rospy.is_shutdown():
        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angle_vel

        rospy.loginfo("Linear Vel = %f: Angular Vel =%f", lin_vel, angle_vel)

        #Publish Twist message
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    move_turtle(float(sys.argv[1]), float(sys.argv[2]))

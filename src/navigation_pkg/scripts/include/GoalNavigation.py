#!/usr/bin/env python3

import rospy  # import rospy
from geometry_msgs.msg import Twist   # import Twist message
from math import atan2, sqrt  # import the mathematical functions atan2 and sqrt from the python module called math
import sys
from std_msgs.msg import Float32MultiArray

class GoalNavigation():
    def __init__(self):
        # rospy.init_node('move_to_goal_node', anonymous=False)        
        # self.sub = rospy.Subscriber('/goal_data', Float32MultiArray, self.pose_callback)        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.vel = Twist()
        self.width = 0
        # rate = rospy.Rate(10)
        self.widthTreshold = 500.
        self.xTreshold = 150.
        self.isReadyToKick = False

    def pose_callback(self,msg):        
        self.width = msg.data[0]
        self.x = abs(msg.data[1])
        move = ""

        print(f"width = {str(self.width)}, x = {str(self.x)}")   

        if (self.width == 0.0) and (self.x ==0.):
            self.selfRotate()
            move = "rotation"
            self.isReadyToKick = False
        if(self.x < self.xTreshold):
            if (self.width < self.widthTreshold and self.width > 0.0):
                self.moveForward(0.08)
                move = "forward"
                self.isReadyToKick = False

            if (self.width >= self.widthTreshold and self.width > 0.0):
                self.stopRobot()
                move = "stop"
                self.isReadyToKick = True
        elif(self.x > self.xTreshold):
            self.selfRotate()
            move = "rotation"
            self.isReadyToKick = False
        # else:            
        #     self.selfRotate()
        #     move = "rotation"
        #     self.isReadyToKick = False
        print(f"width = {str(self.width)}, x = {str(self.x)}, move = {move}")   
    
    def selfRotate(self,angularSpeed=0.5, linearSpeed = -0.05):
        self.vel.angular.z = angularSpeed
        self.vel.linear.x = 0
        self.vel.linear.y = linearSpeed
        # self.pub.publish(self.vel)

    def stopRobot(self):
        self.vel.angular.z = 0
        self.vel.linear.y = 0
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)

    def moveForward(self,linearSpeed):
        self.vel.angular.z = 0
        self.vel.linear.x = linearSpeed
        self.vel.linear.y = 0
        # self.pub.publish(self.vel)

if __name__ == '__main__':
    try:
        NavigationToGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

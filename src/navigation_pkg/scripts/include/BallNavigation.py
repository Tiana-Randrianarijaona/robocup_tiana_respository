#!/usr/bin/env python3

import rospy  # import rospy
from geometry_msgs.msg import Twist   # import Twist message
from math import atan2, sqrt  # import the mathematical functions atan2 and sqrt from the python module called math
import sys
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

class BallNavigation():
    def __init__(self):
        # rospy.init_node('move_to_ball_node', anonymous=False)        
        # self.sub = rospy.Subscriber('/ball_data', Float32MultiArray, self.pose_callback)        
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.vel = Twist()
        self.distance = 0
        # rate = rospy.Rate(10)
        self.distanceTreshold = 40.
        self.xTreshold = 5.
        self.hasNotCaughtTheBall = True

        self.hasSeenTheBall = False
        self.servoAngle = 0

        # Setup GPIO for motor control
        self.dc_motor_pin = 24
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dc_motor_pin, GPIO.OUT)
        GPIO.output(self.dc_motor_pin, GPIO.LOW)

    def pose_callback(self,msg):        
        self.distance = msg.data[0]
        self.x = abs(msg.data[2])
        move = ""
        print(f"Distance = {str(self.distance)}, x = {str(self.x)}")        
        if (self.distance == 0.0 and self.x == 0.0) or (self.distance > self.distanceTreshold and self.x > self.xTreshold):   
            # if self.hasSeenTheBall:
            self.selfRotate(-0.15)
            move = "rotation inversed"                
            self.hasNotCaughtTheBall = True
            # opens the gripper by sending a 0 degree andle to the servo 
            self.servoAngle = 0
            # else:
            #     self.selfRotate(1)
            #     move = "rotation"                
            #     self.hasNotCaughtTheBall = True                        
        elif (self.distance <= self.distanceTreshold and self.distance > 0.0) and (self.x < self.xTreshold and self.x > 0.0):
            self.stopRobot()
            move = "stop"
            self.hasSeenTheBall = False
            self.hasNotCaughtTheBall = False 
            #close the gripper by sending a 70 degree andle to the servo
            self.servoAngle = 70
        # elif (self.x < self.xTreshold and self.x > 0.0):
        #     self.stopRobot()
        #     move = "stop"
            self.hasSeenTheBall = True
            self.hasNotCaughtTheBall = True            
        elif self.distance > self.distanceTreshold and (self.x < self.xTreshold and self.x > 0.0):            
            self.moveForward(0.08)
            self.servoAngle = 0
            move = "forward"
            self.hasSeenTheBall = True
            self.hasNotCaughtTheBall = True
        print(f"Distance = {str(self.distance)}, x = {str(self.x)}, move = {move}, hasNotCaughtTheBall = {self.hasNotCaughtTheBall}")
    
    def selfRotate(self,angularSpeed):
        self.vel.angular.z = angularSpeed
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)
        # rospy.loginfo("STOPPING DC motor.")
        # GPIO.output(self.dc_motor_pin, GPIO.LOW)

    def stopRobot(self):
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)
        # rospy.loginfo("STOPPING DC motor.")
        # GPIO.output(self.dc_motor_pin, GPIO.LOW)
    
    def moveForward(self,linearSpeed):
        self.vel.angular.z = 0
        self.vel.linear.x = linearSpeed
        self.vel.linear.y= 0
        # self.pub.publish(self.vel)
        # rospy.loginfo("STARTING DC motor.")
        # GPIO.output(self.dc_motor_pin, GPIO.HIGH)
# if __name__ == '__main__':
#     try:
#         NavigationToBall()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3

import rospy  # import rospy
from geometry_msgs.msg import Twist   # import Twist message
from math import atan2, sqrt  # import the mathematical functions atan2 and sqrt from the python module called math
import sys
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO

class NavigationToBall():
    def __init__(self):
        rospy.init_node('move_to_ball_node', anonymous=False)        
        self.sub = rospy.Subscriber('/ball_data', Float32MultiArray, self.pose_callback)        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.vel = Twist()
        self.distance = 0
        rate = rospy.Rate(10)
        self.distanceTreshold = 70.
        self.xTreshold = 10.

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
        if self.distance == 0 and self.x == 0:        
            self.selfRotate(0.05)
            move = "rotation"
        elif self.distance < distance_threshold and self.x < x_threshold:
            self.stopRobot()
            move = "stop"
        elif 0 < self.distance <= distance_threshold and self.x > x_threshold::            
            self.moveForward(0.05)
            move = "forward"
        print(f"Distance = {str(self.distance)}, x = {str(self.x)}, move = {move}")  




    
    def selfRotate(self,angularSpeed):
        self.vel.angular.z = angularSpeed
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)
        # Start the motor
        rospy.loginfo("Starting DC motor.")
        GPIO.output(self.dc_motor_pin, GPIO.LOW)

    def stopRobot(self):
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)
        # Start the motor
        rospy.loginfo("Starting DC motor.")
        GPIO.output(self.dc_motor_pin, GPIO.LOW)
    
    def moveForward(self,linearSpeed):
        self.vel.angular.z = 0
        self.vel.linear.x = linearSpeed
        # self.pub.publish(self.vel)
        # Start the motor
        rospy.loginfo("Starting DC motor.")
        GPIO.output(self.dc_motor_pin, GPIO.HIGH)
if __name__ == '__main__':
    try:
        NavigationToBall()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

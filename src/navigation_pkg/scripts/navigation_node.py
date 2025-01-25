#!/usr/bin/env python3

import rospy  # import rospy
from geometry_msgs.msg import Twist   # import Twist message
from math import atan2, sqrt  # import the mathematical functions atan2 and sqrt from the python module called math
import sys
from std_msgs.msg import Float32MultiArray, Bool
from std_msgs.msg import Int32
import sys
import os
# sys.path.append(os.path.abspath("../include"))
from include.BallNavigation import BallNavigation
from include.GoalNavigation import GoalNavigation
from include.ColleagueNavigation import ColleagueNavigation

class Navigation():
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)        
        self.ballNavigator = BallNavigation()
        # self.goalNavigator = GoalNavigation()
        self.colleagueNavigator = ColleagueNavigation()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.pubGripper = rospy.Publisher('/servo_angle', Int32, queue_size = 100)
        self.kickerPub = rospy.Publisher('/kicking_decision', Bool, queue_size = 10)
        self.ballSub = rospy.Subscriber('/ball_data', Float32MultiArray, self.ball_callback)     
        # self.goalSub = rospy.Subscriber('/goal_data', Float32MultiArray, self.goal_callback)           
        self.colleagueSub = rospy.Subscriber('/colleague_data', Float32MultiArray, self.colleague_callback)           
        self.vel = Twist()    
        self.kickingDecision = Bool()    
        rate = rospy.Rate(50)        

    def ball_callback(self,msg):  
        self.ballNavigator.pose_callback(msg)
        if self.ballNavigator.hasNotCaughtTheBall:            
            # self.pub.publish(self.ballNavigator.vel)
            self.pubGripper.publish(self.ballNavigator.servoAngle)
            
        
    # def goal_callback(self,msg) :
    #     if not (self.ballNavigator.hasNotCaughtTheBall):            
    #         self.goalNavigator.pose_callback(msg)
    #         self.notifyKickerNode(self.goalNavigator.isReadyToKick)
    #         print(f"IsReadyToKick = {self.goalNavigator.isReadyToKick}")
    #         self.pub.publish(self.goalNavigator.vel)

    def colleague_callback(self,msg) :
        # if not (self.ballNavigator.hasNotCaughtTheBall):            
        self.colleagueNavigator.pose_callback(msg)
        # self.notifyKickerNode(self.goalNavigator.isReadyToKick)
        # print(f"IsReadyToKick = {self.goalNavigator.isReadyToKick}")
        self.pub.publish(self.colleagueNavigator.vel)

    def notifyKickerNode(self,kickingDecision):
        self.kickingDecision.data = kickingDecision
        self.kickerPub.publish(self.kickingDecision)
if __name__ == '__main__':
    try:
        Navigation()
        rospy.spin()    
    except rospy.ROSInterruptException:
        pass

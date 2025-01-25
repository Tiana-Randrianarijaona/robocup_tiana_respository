#!/usr/bin/env python3

import rospy  # import rospy
from geometry_msgs.msg import Twist  # import Twist message
from std_msgs.msg import Float32MultiArray, Bool, Int32
from include.BallNavigation import BallNavigation
from include.ColleagueNavigation import ColleagueNavigation

class Navigation():
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        
        # Instantiate BallNavigation and ColleagueNavigation
        self.ballNavigator = BallNavigation()
        self.colleagueNavigator = ColleagueNavigation()

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubGripper = rospy.Publisher('/servo_angle', Int32, queue_size=100)
        self.kickerPub = rospy.Publisher('/kicking_decision', Bool, queue_size=10)
        self.ballSub = rospy.Subscriber('/ball_data', Float32MultiArray, self.ball_callback)
        self.colleagueSub = rospy.Subscriber('/colleague_data', Float32MultiArray, self.colleague_callback)

        # State flags
        self.searchingForBall = True
        self.readyToKick = False
        self.vel = Twist()
        rospy.loginfo("Navigation node initialized")

    def ball_callback(self, msg):
        """Handle ball detection and navigation."""
        if self.searchingForBall:
            self.ballNavigator.pose_callback(msg)

            # Publish ball navigation commands
            self.pub.publish(self.ballNavigator.vel)
            self.pubGripper.publish(self.ballNavigator.servoAngle)

            # Check if the ball has been "caught"
            if not self.ballNavigator.hasNotCaughtTheBall:
                rospy.loginfo("Ball caught! Transitioning to ArUco detection.")
                self.searchingForBall = False  # Switch to ArUco detection

    def colleague_callback(self, msg):
        """Handle ArUco detection and navigation."""
        if not self.searchingForBall:
            self.colleagueNavigator.pose_callback(msg)

            # Publish ArUco navigation commands
            self.pub.publish(self.colleagueNavigator.vel)

            # Check if the ball is still on the servo
            if self.ballNavigator.hasNotCaughtTheBall:
                rospy.logwarn("Ball lost! Returning to ball detection.")
                self.searchingForBall = True  # Switch back to ball detection
                return

            # Check if the robot is aligned with the ArUco code
            if not self.colleagueNavigator.hasNotCaughtTheAruco:
                rospy.loginfo("Aligned with ArUco! Ready to kick.")
                self.readyToKick = True
                self.pubGripper.publish(0)
                self.trigger_kick()


    def trigger_kick(self):
        """Trigger the kicker when conditions are met."""
        if self.readyToKick:
            rospy.loginfo("Kicking the ball!")
            kick_command = Bool()
            kick_command.data = True
            self.kickerPub.publish(kick_command)

            # Reset states after the kick
            self.readyToKick = False
            self.searchingForBall = True  # Prepare for the next ball

    def notifyKickerNode(self, kickingDecision):
        """Notify the kicker node if ready to kick."""
        self.kickingDecision.data = kickingDecision
        self.kickerPub.publish(self.kickingDecision)

if __name__ == '__main__':
    try:
        Navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

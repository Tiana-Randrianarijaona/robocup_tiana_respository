#!/usr/bin/env python3

import rospy  # Import rospy
from geometry_msgs.msg import Twist  # Import Twist message
from std_msgs.msg import Float32MultiArray  # Import Float32MultiArray

class BallNavigation:
    def __init__(self):
        # Initialize variables
        self.vel = Twist()
        self.x = 0.0  # X-coordinate of the ball center
        self.y = 0.0  # Y-coordinate of the ball center
        self.x_center_min = 200  # Minimum X value for the center of the frame
        self.x_center_max = 400  # Maximum X value for the center of the frame
        self.y_stop_threshold = 290  # Y value to stop the robot
        self.y_forward_threshold = 300  # Y value to move the robot forward
        self.hasNotCaughtTheBall = True
        self.hasSeenTheBall = False
        self.servoAngle = 0  # Initial servo angle

    def pose_callback(self, msg):
        """Process ball position data."""
        self.x = msg.data[0]  # X offset of the ball from the frame center
        self.y = msg.data[1]  # Y offset of the ball from the frame center

        # Ignore invalid data where x or y is 0
        # if self.x == 0.0 or self.y == 0.0:
        #     rospy.loginfo("Invalid ball data received, skipping.")
        #     return

        move = ""
        rospy.loginfo(f"x = {self.x}, y = {self.y}")

        # Check if the ball is within the center area
        if self.x_center_min <= self.x <= self.x_center_max:
            if self.y >= self.y_stop_threshold:  # Stop the robot
                self.stopRobot()
                move = "stop"
                self.hasSeenTheBall = False
                self.hasNotCaughtTheBall = False
                self.servoAngle = 70  # Close the gripper to pick up the ball
            elif self.y < self.y_stop_threshold:  # Move forward
                self.moveForward(0.08)
                move = "move forward"
                self.servoAngle = 0
                self.hasNotCaughtTheBall = True
        else:  # Ball is outside the center area
            self.selfRotate(0.5)  # Rotate right
            move = "rotate right to center the ball"    

        rospy.loginfo(f"x = {self.x}, y = {self.y}, move = {move}, hasNotCaughtTheBall = {self.hasNotCaughtTheBall}, servoAngle = {self.servoAngle}")

    def selfRotate(self, angularSpeed):
        """Rotate the robot."""
        self.vel.angular.z = angularSpeed
        self.vel.linear.x = 0
        # Publish velocity command (uncomment when ROS is active)
        # self.pub.publish(self.vel)

    def stopRobot(self):
        """Stop the robot."""
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        # Publish velocity command (uncomment when ROS is active)
        # self.pub.publish(self.vel)

    def moveForward(self, linearSpeed):
        """Move the robot forward."""
        self.vel.angular.z = 0
        self.vel.linear.x = linearSpeed
        self.vel.linear.y = 0
        # Publish velocity command (uncomment when ROS is active)
        # self.pub.publish(self.vel)

# Uncomment the following lines when integrating with ROS
# if __name__ == '__main__':
#     try:
#         rospy.init_node('move_to_ball_node', anonymous=False)
#         ball_navigation = BallNavigation()
#         rospy.Subscriber('/ball_data', Float32MultiArray, ball_navigation.pose_callback)
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

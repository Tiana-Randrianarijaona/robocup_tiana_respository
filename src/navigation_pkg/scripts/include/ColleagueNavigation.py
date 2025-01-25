#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class ColleagueNavigation():
    def __init__(self):
        # rospy.init_node('move_to_ball_node', anonymous=False)
        # self.sub = rospy.Subscriber('/aruco_data', Float32MultiArray, self.pose_callback)
        # self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.vel = Twist()
        self.cx = 0
        self.cy = 0
        self.central_area_threshold_x = 50  # Threshold for x-axis
        self.central_area_threshold_y = 50  # Threshold for y-axis
        self.hasNotCaughtTheAruco = True

    def pose_callback(self, msg):
        # Extract cx and cy from the message data
        self.cx = msg.data[0]
        self.cy = msg.data[1]
        move = ""

        # Create a blank image to represent the camera feed (for testing)
        camera_width, camera_height = 640, 480
        frame = np.zeros((camera_height, camera_width, 3), dtype=np.uint8)

        # Define the central area for the Aruco code (where the robot should stop)
        center_x = camera_width // 2
        center_y = camera_height // 2

        # Define top-left and bottom-right points of the central region rectangle
        top_left = (center_x - self.central_area_threshold_x, center_y - self.central_area_threshold_y)
        bottom_right = (center_x + self.central_area_threshold_x, center_y + self.central_area_threshold_y)

        # Draw the central area rectangle (where the Aruco code should be located)
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        # Draw a circle where the Aruco code is currently detected
        cv2.circle(frame, (int(self.cx), int(self.cy)), 10, (0, 0, 255), -1)

        print(f"cx = {self.cx}, cy = {self.cy}")

        # Check if the Aruco code is within the central area
        if abs(self.cx - center_x) <= self.central_area_threshold_x and abs(self.cy - center_y) <= self.central_area_threshold_y:
            self.stopRobot()
            move = "stop"
            self.hasNotCaughtTheAruco = False
        else:
            # Determine whether to rotate or move forward based on the Aruco code position
            if self.cx < center_x - self.central_area_threshold_x:
                self.selfRotate(0.15)  # Rotate left
                move = "rotate left"
            elif self.cx > center_x + self.central_area_threshold_x:
                self.selfRotate(-0.15)  # Rotate right
                move = "rotate right"

            # If within the x threshold but not centered on y, move forward
            if abs(self.cx - center_x) <= self.central_area_threshold_x:
                self.moveForward(0.15)
                move = "forward"

            self.hasNotCaughtTheAruco = True

        print(f"cx = {self.cx}, cy = {self.cy}, move = {move}")

        

    def selfRotate(self, angularSpeed):
        self.vel.angular.z = angularSpeed
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)

    def stopRobot(self):
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        # self.pub.publish(self.vel)

    def moveForward(self, linearSpeed):
        self.vel.angular.z = 0
        self.vel.linear.x = linearSpeed
        # self.pub.publish(self.vel)

# if __name__ == '__main__':
#     try:
#         ColleagueNavigation()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

class DistanceCalculator:
    def __init__(self):
        rospy.init_node('distance_calculator_node', anonymous=True)
        self.odom_sub = rospy.Subscriber('/raw_odom', Odometry, self.odom_callback)
        self.last_position = None
        self.total_distance = 0.0

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        # print("current_position = {current_position}")

        if self.last_position is not None:
            distance = self.calculate_distance(self.last_position, current_position)
            self.total_distance += distance

        self.last_position = current_position

        print("Total distance traveled: %.2f meters" % self.total_distance)

    def calculate_distance(self, point1, point2):
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z

        return math.sqrt(dx*dx + dy*dy + dz*dz)

if __name__ == '__main__':
    try:
        distance_calculator = DistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ball_tracking_hafa import Tracker

class Image_processor():
    def __init__(self):
        self.bridge = CvBridge()
        self.tracker = Tracker()  
        rospy.init_node('ball_tracking_node')
        image_topic = "/image_raw"
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback) 
        self.pub = rospy.Publisher('/ball_detection', Image, queue_size=1)           
        self.pubFloats = rospy.Publisher('/ball_data', Float32MultiArray, queue_size=1)   
        rate = rospy.Rate(10)
        rospy.spin()

    def image_callback(self, msg):
        # Convert the image message to a cv2 image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Call the Tracker callback function to process the frame and get the ball's x and y position
        result = self.tracker.callBack(frame)
        # print(result)
        
        # If the ball is not detected, result will have x_real and y_real as 0
        if result["x_real"] == 0 and result["y_real"] == 0:
            rospy.logwarn("Ball not detected!")
        
        # Publish the processed frame with visual feedback
        self.pub.publish(self.bridge.cv2_to_imgmsg(result["frame"], "bgr8"))
        
        # Publish both x_real and y_real values for controlling robot movement
        msg = Float32MultiArray()
        msg.data = [result["x_real"], result["y_real"]]
        self.pubFloats.publish(msg)


if __name__ == '__main__':
    image_det = Image_processor()

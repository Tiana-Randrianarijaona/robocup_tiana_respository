#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os

# Get the path to the ../include directory
include_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../include'))

# Add that directory to the     .path
sys.path.append(include_path)

# Now you can import the class
from ColleagueFinder import ColleagueFinder

class ColleagueTrackerNode():
    def __init__(self,targetMarkerId):
        self.bridge = CvBridge()
        self.colleagueFinder = ColleagueFinder()  
        rospy.init_node('colleague_tracking_node')
        image_topic = "image_raw"
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback) 
        self.pub = rospy.Publisher('/colleague_detection', Image, queue_size=1)           
        self.pubFloats = rospy.Publisher('/colleague_data', Float32MultiArray, queue_size=1)   
        self.targetMarkerId = targetMarkerId
        rate = rospy.Rate(10)
        rospy.spin()
          

    def image_callback(self,msg):    
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = np.array(cv2_img, dtype=np.uint8)
        # get the frame, the distance between the robot and the ball, and the in_frame_radius of the ball
        result = self.colleagueFinder.callback(frame,self.targetMarkerId)        
        # publish the image showing the ball being detected
        self.pub.publish(self.bridge.cv2_to_imgmsg(result['frame'],"bgr8"))        
        msg = Float32MultiArray()
        msg.data = [result['cx'], result['cy']]
        # publish the distance and the radius
        self.pubFloats.publish(msg)
        
if __name__ == '__main__':
    image_det = ColleagueTrackerNode(1)   
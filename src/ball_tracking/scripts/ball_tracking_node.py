#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ball_tracking import Tracker

class Image_processor():
    def __init__(self):
        self.bridge = CvBridge()
        self.tracker = Tracker()  
        rospy.init_node('ball_tracking_node')
        image_topic = "/image_raw"
        self.sub = rospy.Subscriber(image_topic,Image, self.image_callback) 
        self.pub = rospy.Publisher('/ball_detection', Image, queue_size=1)           
        self.pubFloats = rospy.Publisher('/ball_data', Float32MultiArray, queue_size=1)   
        rate = rospy.Rate(10)
        rospy.spin()
          

    def image_callback(self,mdg):    
        # cv2_img = self.bridge.imgmsg_to_cv2(mdg, "bgr8")
        cv2_img = self.bridge.imgmsg_to_cv2(mdg, "bgr8")
        
        frame = np.array(cv2_img, dtype=np.uint8)
        # get the frame, the distance between the robot and the ball, and the in_frame_radius of the ball
        result = self.tracker.callBack(frame)
        # publish the image showing the ball being detected
        self.pub.publish(self.bridge.cv2_to_imgmsg(result['frame'],"bgr8"))
        msg = Float32MultiArray()
        msg.data = [result['distance'], result['radius'], result['x']]
        # publish the distance and the radius
        self.pubFloats.publish(msg)
        

    # def edges_detection(frame):
    #     grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #     blurred = cv2.blur(grey, (7,7))
    #     edges = cv2.Canny(blurred, 15.0, 30.0)
    #     # cv2.imshow("color", edges)
    #     return edges

    # def color_detection(frame):
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #     # blue_lower = np.array([90,50,50], np.uint8)
    #     # blue_upper = np.array([120,255,255], np.uint8)
    #     # lower_red = np.array([0,50,50], np.uint8)
    #     # upper_red = np.array([10,255,255], np.uint8)
    #     # lower_yellow = np.array([22, 93, 0])
    #     # upper_yellow = np.array([45, 255, 255])
    #     lower_yellow = np.array([22, 100, 0])
    #     upper_yellow = np.array([45, 255, 255])
    #     mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #     cv2.imshow("image mask", mask)
    #     res = cv2.bitwise_and(frame, frame, mask=mask)
    #     cv2.imshow("color", res)
    #     edge = edges_detection(res)
    #     cv2.imshow("edge", edge)
    #     return res

    

if __name__ == '__main__':
    image_det = Image_processor()   
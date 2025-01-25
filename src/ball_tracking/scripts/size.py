#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        
    def callback(self, data):
        try:
            # Convert the ROS Image message to a CV2 Image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Get the dimensions of the frame
        height, width = cv_image.shape[:2]
        print(f"Frame size: Width = {width}, Height = {height}")

        # Display the frame (optional)
        cv2.imshow('Frame', cv_image)
        cv2.waitKey(3)

def main():
    # Initialize the ROS node
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

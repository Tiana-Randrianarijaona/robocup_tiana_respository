# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import cv2
import imutils
import time
from TrackerHelper import TrackerHelper

yellow_min = (21,116,106)     
yellow_max = (31,255,255)
class Tracker():
	def __init__(self,
			  trackerHelper = TrackerHelper(real_ball_radius=3.75, focal_length=750.),
			#   yellowLower = (16,34,144),
			#   yellowUpper = (43, 151, 255)):	
			# yellowLower = (102,91,90),
			# yellowUpper = (255,255,255)	):
			#for blue ball
			# yellowLower = (88,102,120),
			# yellowUpper = (97,210,255)	):
			#for Green balls
				yellowLower = (16,93,70),
				yellowUpper = (68,177,255)	):	
		# trackerHelper = TrackerHelper(real_ball_radius=5., focal_length=277.)
		self.trackerHelper = trackerHelper

		# define the lower and upper boundaries of the "green"
		# ball in the HSV color space, then initialize the
		# list of tracked points
		# greenLower = (29, 86, 6)
		# greenUpper = (64, 255, 255)
		self.yellowLower = yellowLower
		self.yellowUpper = yellowUpper
		self.buffer = 64
		self.pts = deque(maxlen=self.buffer)


		# allow the camera or video file to warm up
		time.sleep(2.0)

	def callBack(self, frame_from_image_topic):
		radius = 0
		distance = 0
		x_real = 0
		# grab the current frame
		frame = frame_from_image_topic

		# resize the frame, blur it, and convert it to the HSV
		# color space
		# frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, self.yellowLower, self.yellowUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)
		center = None
		rows = frame.shape[0]
		cols = frame.shape[1]
		size = min([rows, cols])
		center_x = int(cols/2.0)
		center_y = int(rows/2.0)
		# print(f"shape: {frame.shape}")	
		
		line_length = int(size*0.3)
		
		#-- X
		frame = cv2.line(frame, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), 2)
		#-- Y
		frame = cv2.line(frame, (center_x, center_y), (center_x, center_y+line_length), (0,255,0), 2)				

		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

			# only proceed if the radius meets a minimum size
			if radius > 0:
				# draw the circle and centroid on the frame,
				# then update the list of tracked points
				cv2.circle(frame, (int(x), int(y)), int(radius),
					(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
				# self.trackerHelper.setFocalLength(30.,radius)
				# print(f"focal_length = {self.trackerHelper.focal_length}")				
				distance, x_real = self.trackerHelper.getCoordinates(radius,x,center_x)
				# print(f"Distance = {distance},Radius = {radius}")			

		# update the points queue
		self.pts.appendleft(center)

		# loop over the set of tracked points
		for i in range(1, len(self.pts)):
			# if either of the tracked points are None, ignore
			# them
			if self.pts[i - 1] is None or self.pts[i] is None:
				continue

			# otherwise, compute the thickness of the line and
			# draw the connecting lines
			thickness = int(np.sqrt(self.buffer / float(i + 1)) * 2.5)
			cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

		# show the frame to our screen
		#cv2.imshow("Ball detection", frame)
		return {"frame": frame, "distance":distance, "radius":radius, "x":x_real}		

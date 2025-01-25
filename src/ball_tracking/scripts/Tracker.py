class Tracker():
    def __init__(self, yellowLower=(16, 93, 70), yellowUpper=(68, 177, 255)):
        self.yellowLower = yellowLower
        self.yellowUpper = yellowUpper
        self.buffer = 64
        self.pts = deque(maxlen=self.buffer)

    def callBack(self, frame_from_image_topic):
        center = None
        # grab the current frame
        frame = frame_from_image_topic

        # resize the frame, blur it, and convert it to the HSV color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "yellow" (or whatever color ball you're tracking)
        mask = cv2.inRange(hsv, self.yellowLower, self.yellowUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
        # Get the frame dimensions and center
        rows = frame.shape[0]
        cols = frame.shape[1]
        center_x = int(cols / 2.0)
        center_y = int(rows / 2.0)
        
        # add the crosshair for visual feedback
        line_length = int(min(rows, cols) * 0.3)
        frame = cv2.line(frame, (center_x, center_y), (center_x + line_length, center_y), (0, 0, 255), 2)
        frame = cv2.line(frame, (center_x, center_y), (center_x, center_y + line_length), (0, 255, 0), 2)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour and compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius is greater than 0 (this filters out small/noise)
            if radius > 0:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # calculate x_real and y_real as the offset from the center of the frame
        x_real, y_real = 0, 0
        if center is not None:
            x_real = center[0] - center_x
            y_real = center[1] - center_y

        # Update the list of tracked points
        self.pts.appendleft(center)

        # Draw the tracking trail
        for i in range(1, len(self.pts)):
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue
            thickness = int(np.sqrt(self.buffer / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        # Ensure that x_real and y_real are always returned
        return {"frame": frame, "x_real": x_real, "y_real": y_real}

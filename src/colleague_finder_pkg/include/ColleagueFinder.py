import apriltag
import cv2
from ArucoMarker import ArucoMarker


class ColleagueFinder:
    def __init__(self):
        # Initialize the AprilTags detector options
        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)
        self.central_area_threshold_x = 50  # Threshold for x-axis
        self.central_area_threshold_y = 50  # Threshold for y-axis

    #This method is the callback function to be executed whenever the frame from the camera is received
    def callback(self, frame_from_camera, targetMarkerId):
        markers = self.getColleagueInfo(frame_from_camera)
        frame_with_markers = self.markColleagueInFrame(frame_from_camera,markers)
        #return the info Arucomarker info if the targetted Id is detected
        is_target_ip_detected = False
        targetMarker = ArucoMarker(False,0,0)
        for marker in markers:
            if marker.id == targetMarkerId:
                is_target_ip_detected = True
                targetMarker = marker
                break
        return {"frame": frame_with_markers, 
                "is_target_ip_detected": is_target_ip_detected,
                "cx": targetMarker.cx,
                "cy": targetMarker.cy}


    def getColleagueInfo(self, frame):
        """
        Detects AprilTags in the given frame and returns a list of ArucoMarker objects.
        :param frame: Video frame in which AprilTags need to be detected.
        :return: List of ArucoMarker objects containing ID and center coordinates.
        """
        # Convert the frame to grayscale (AprilTag detection works on grayscale images)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect the AprilTags in the frame
        results = self.detector.detect(gray)

        # List to hold detected ArucoMarker instances
        markers = []

        # Loop over the AprilTag detection results
        for r in results:
            # Get the tag ID and center coordinates
            tagID = r.tag_id
            (cx, cy) = (int(r.center[0]), int(r.center[1]))

            # Create an ArucoMarker instance for each detected tag
            marker = ArucoMarker(marker_id=tagID, cx=cx, cy=cy)

            # Add the marker to the list
            markers.append(marker)

        return markers
    
    # draw green rectangle on the Aruco code with its marker ID and a red button in its center
    # return the resulting frame
    def markColleagueInFrame(self,frame,markers):
        # Loop over each detected marker and display its information
        for marker in markers:
            # Print the marker details (optional)
            marker.describe()

            # Draw a red dot at the center of the marker (cx, cy)
            cv2.circle(frame, (marker.cx, marker.cy), 5, (0, 0, 255), -1)  # red dot at (cx, cy)

            # Display the marker ID next to the red dot
            cv2.putText(frame, f"ID: {marker.id}", (marker.cx - 10, marker.cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Create a blank image to represent the camera feed (for testing)
        camera_width, camera_height = 640, 480
        # frame = np.zeros((camera_height, camera_width, 3), dtype=np.uint8)

        # Define the central area for the Aruco code (where the robot should stop)
        center_x = camera_width // 2
        center_y = camera_height // 2

        # Define top-left and bottom-right points of the central region rectangle
        top_left = (center_x - self.central_area_threshold_x, center_y - self.central_area_threshold_y)
        bottom_right = (center_x + self.central_area_threshold_x, center_y + self.central_area_threshold_y)

        # Draw the central area rectangle (where the Aruco code should be located)
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)

        # Draw a circle where the Aruco code is currently detected
        # cv2.circle(frame, (int(self.cx), int(self.cy)), 10, (0, 0, 255), -1)

        
        return frame

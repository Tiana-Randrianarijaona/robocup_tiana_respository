class ArucoMarker:
    def __init__(self, marker_id, cx, cy):
        # Initialize the ArucoMarker instance with its ID and center coordinates
        self.id = marker_id
        self.cx = cx
        self.cy = cy

    def describe(self):
        # Method to print the marker's attributes
        print(f"Aruco Marker ID: {self.id}, Center X: {self.cx}, Center Y: {self.cy}")

class TrackerHelper():
    def __init__(self, focal_length,real_ball_radius):
        self.focal_length = focal_length
        self.real_ball_radius = real_ball_radius

    # focal length finder function
    def setFocalLength(self,measured_distance, radius_in_frame):

        focal_length_value = (radius_in_frame * measured_distance) / self.real_ball_radius
        #return focal length.
        self.focal_length =  focal_length_value
    
    #get the distance of the ball from the camera and its coordinate along the horizontal axis (x axis)
    def getCoordinates(self, ball_radius_in_frame,x_in_frame, frame_center_along_x_axis):
        distance_from_camera = (self.real_ball_radius * self.focal_length)/ball_radius_in_frame
        x = ((x_in_frame - frame_center_along_x_axis) * distance_from_camera )/self.focal_length
        print(f"(distance_from_camera,x, ball_radius) = ({distance_from_camera},{x},{self.real_ball_radius})")
        return (distance_from_camera,x)   

    def getDistance(self,radius_in_frame):
        return self.focal_length * self.real_ball_radius / radius_in_frame
    # def initializeFocalLength(self):
        

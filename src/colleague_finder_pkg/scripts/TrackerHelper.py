class TrackerHelper():
    def __init__(self, focal_length = 10000,real_ball_radius = 100):
        self.focal_length = focal_length
        self.real_ball_radius = real_ball_radius

    # focal length finder function
    def setFocalLength(self,measured_distance, radius_in_frame):

        focal_length_value = (radius_in_frame * measured_distance) / self.real_ball_radius
        #return focal length.
        self.focal_length =  focal_length_value
    
    #get the in frame distance from the center of the camera and the center of the goal
    def getCoordinates(self,x_in_frame, frame_center_along_x_axis):
        x = (x_in_frame - frame_center_along_x_axis)
        print(f"x_goal = {x_in_frame}, x_camera = {frame_center_along_x_axis}, distance = {x}")
        return (x)   
 

    def getDistance(self,radius_in_frame):
        return self.focal_length * self.real_ball_radius / radius_in_frame
    # def initializeFocalLength(self):
        

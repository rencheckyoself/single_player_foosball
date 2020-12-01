bool rod_is_up # true if the rod is raised so that the ball can pass under
bool players_are_back # true if the rod is raised with the players feet closest to their own goal
sensor_msgs/RegionOfInterest bounding_rect_roi # bounding rectangle for the tracked player on the rod for the cropped image
sensor_msgs/RegionOfInterest bounding_rect_img # bounding rectangle for the tracked player on the rod for the native image

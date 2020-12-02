bool rod_is_up # true if the rod is raised so that the ball can pass under
bool players_are_back # true if the rod is raised with the players feet closest to their own goal
float32 min_x_value # the minimum x value of the bounding rectangle.
sensor_msgs/RegionOfInterest bounding_rect_img # bounding rectangle for the tracked player on the rod for the native image
sensor_msgs/RegionOfInterest window_roi # the window to crop the main image to for tracking the rod angle

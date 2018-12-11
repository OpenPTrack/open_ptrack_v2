# This message is used to transfer the camera image from ARCore devices
# together with the pose estimated by ARcore and the camera intrinsic
# parameters.

Header header

# The pose of the camera estimated by ARcore
geometry_msgs/Pose mobileFramePose 

# The camera image
sensor_msgs/CompressedImage image 

# The focal lengths in pixels
float64 focal_length_x_px	
float64 focal_length_y_px

# The image size in pixels
int32 image_width_px
int32 image_height_px

# The position of the principal point, in pixels
float64 principal_point_x_px
float64 principal_point_y_px
 
# The pixel size in millimiters
float64 pixel_size_x_mm
float64 pixel_size_y_mm


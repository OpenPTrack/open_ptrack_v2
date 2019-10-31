# This message represents the registration of a specific AR Device
# coordinate system with respect to the ROS world frame
#
# This message is used for transfering the raw frame-by-frame unfiltered
# estimations to th aggregator node, which will aggregate the estimations
# from the different cameras and filter them
#
# author: Carlo Rizzardo (c-rizz)

Header header

# The transform representing the registration between the two systems
geometry_msgs/TransformStamped transform

# Unique identifier of the AR device
string deviceId

# Fixed camera used to compute this estimation
string fixed_sensor_name

# Number of matches found between the camera image and the AR camera image.
# More matches imply a more realiable estimate as these are used to compute the AR device pose with PnP
int32 matches_number

# Reprojection error of the matches using the estimated pose. Usually less then 2 pixels for good estimations
float64 reprojection_error

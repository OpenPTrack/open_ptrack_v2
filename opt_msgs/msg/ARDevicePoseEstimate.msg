# This message represents the pose estimate for an ARDevice obtained from a single
# PnP-based estimation. Together with some data useful for quantifying the realiability
# of the estimate
#
# This message is used for transfering the raw frame-by-frame PnP estimates of the
# ARDevice poses from the single-camera nodes to the aggregator node
#
# author: Carlo Rizzardo (c-rizz)

Header header

# The estimated camera pose
geometry_msgs/PoseStamped cameraPose

# The camera pose in the ARCore frame
geometry_msgs/Pose cameraPose_mobileFrame

# Unique identifier of the AR device
string deviceId

# Fixed camera used to compute this estimation
string fixed_sensor_name

# Number of matches found between the camera image and the AR camera image.
# More matches imply a more realiable estimate as these are used to compute the AR device pose with PnP
int32 matches_number

# Reprojection error of the matches using the estimated pose. Usually less then 2 pixels for good estimations
float64 reprojection_error

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

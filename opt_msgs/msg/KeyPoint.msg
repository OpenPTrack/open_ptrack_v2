# ROS message used to transfer an OpenCV cv::KeyPoint object
#
# author: Carlo Rizzardo (c-rizz)


# The y pixel position of the keypoint in the image
float32 xPos;

# The y pixel position of the keypoint in the image
float32 yPos;

# The angle of the keypoint
float32 angle;

# object class (if the keypoints need to be clustered by an object they belong to)
int32 class_id;

# octave (pyramid layer) from which the keypoint has been extracted
int32 octave;

# the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
float32 response;

# diameter of the meaningful keypoint neighborhood
float32 size;

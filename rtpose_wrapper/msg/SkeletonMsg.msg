##### Add your skeleton unique id here
##### and remember to set the skeleton_type field accordingly
##### Remember also to define how to return a centroid in the
##### SkeletonTracker class

uint8 MPI = 0
uint8 COCO = 1

Header header
rtpose_wrapper/Joint3DMsg[] joints
string info
uint8 skeleton_type
float64 height
float64 distance
float64 confidence
bool occluded

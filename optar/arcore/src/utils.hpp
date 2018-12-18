#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>

cv::Point3f get3dPoint(int x, int y, const cv::Mat& depthImage, double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);
cv::Mat DoubleMatFromVec3b(cv::Vec3b in);
void opencvPoseToEigenPose(cv::Vec3d tvecV, cv::Vec3d rvecV, Eigen::Vector3d &Translate, Eigen::Quaterniond &quats);
int publish_pose_for_viewing(std_msgs::Header header, float tx, float ty, float tz, float qx, float qy, float qz, float qw, ros::Publisher pose_marker_pub);

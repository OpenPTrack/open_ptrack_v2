#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <visualization_msgs/Marker.h>
#include <opencv2/core/core.hpp>



//functions documentation in utils.cpp

cv::Point3f get3dPoint(int x, int y, int depth_mm, double focalLengthX, double focalLengthY, double principalPointX, double principalPointY);
cv::Mat DoubleMatFromVec3b(cv::Vec3b in);
void opencvPoseToEigenPose(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Vector3d &translation, Eigen::Quaterniond &quaternion);
int publish_pose_for_viewing(float tx, float ty, float tz, float qx, float qy, float qz, float qw, ros::Publisher pose_marker_pub, std::string name, float r, float g, float b, float a, float size);
double poseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
int buildMarker(visualization_msgs::Marker& marker_pose, const geometry_msgs::Pose& pose, std::string name, float r, float g, float b, float a, float size, std::string frame_id);
int buildMarker(visualization_msgs::Marker& marker_pose, const cv::Point3f& position, std::string name, float r, float g, float b, float a, float size, std::string frame_id);
cv::Point2i findNearestNonZeroPixel(const cv::Mat& image, int x, int y, double maxDist);
void transformCvPoint3f(const cv::Point3f& in, cv::Point3f& out, tf::StampedTransform transform);
void prepareOpencvImageForShowing(std::string winName, cv::Mat image, int winHeight, int winWidth=-1);
void publishPoseAsTfFrame(geometry_msgs::PoseStamped& pose, std::string tfFrameName);

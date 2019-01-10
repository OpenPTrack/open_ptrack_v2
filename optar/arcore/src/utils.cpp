
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xphoto.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;


/**
 * Get 3d coordinates from the image pixel coordinates and the depth image
 * @param x pixel x position
 * @param y pixel y position
 * @param depthImage x the depth image
 * @param focalLengthX focal length on the x axis
 * @param focalLengthY focal length on the y axis
 * @param principalPointX principal point x coordinate
 * @param principalPointY principal point y coordinate
 *
 * @return the 3d point
 */
cv::Point3f get3dPoint(int x, int y, int depth_mm, double focalLengthX, double focalLengthY, double principalPointX, double principalPointY)
{
	cv::Point3f p;
	p.z = (float)(((double)depth_mm)/1000);//convert to meters
	p.x = (x - principalPointX) * p.z / focalLengthX;
	p.y = (y - principalPointY) * p.z / focalLengthY;
	//ROS_INFO_STREAM("depth = "<<depth_mm);
	return p;
}

/**
 * Converts an opencv Vec3b to a opencv Mat
 * @param in the vector
 *
 * @return the Mat
 */
cv::Mat DoubleMatFromVec3b(cv::Vec3b in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
};

/**
 * Converts a pose expressed as an opencv position vector and an opencv orientation vector
 * to a pose expressed as an Eigen vector and an Eigen quateronion
 * @param tvec opencv position vector
 * @param rvecV opencv rotation vector
 * @param Translate Eigen position vector
 * @param quats Eigen quaternion
 */
void opencvPoseToEigenPose(cv::Vec3d tvecV, cv::Vec3d rvecV, Eigen::Vector3d &Translate, Eigen::Quaterniond &quats)
{
    cv::Mat R;
    cv::Mat tvec, rvec;

    tvec = DoubleMatFromVec3b(tvecV);
    rvec = DoubleMatFromVec3b(rvecV);

    cv::Rodrigues(rvec, R); // R is 3x3
    R = R.t();                 // rotation of inverse
    tvec = -R*tvec;           // translation of inverse

    Eigen::Matrix3d mat;
    cv2eigen(R, mat);

    Eigen::Quaterniond EigenQuat(mat);

    quats = EigenQuat;


    double x_t = tvec.at<double>(0, 0);
    double y_t = tvec.at<double>(1, 0);
    double z_t = tvec.at<double>(2, 0);

    Translate.x() = x_t * 10;
    Translate.y() = y_t * 10;
    Translate.z() = z_t * 10;   

}

/**
 * Publishes a marker for rviz and a tf frame for this pose
 * @param header the header from the pose
 * @param tx position x
 * @param ty position y
 * @param tz position z
 * @param qx orientation quaternion x component
 * @param qy orientation quaternion y component
 * @param qz orientation quaternion z component
 * @param qw orientation quaternion w component
 * @param pose_marker_pub publisher for the marker, of visualization_msgs::Marker type
 *
 * @return zero if successful
 */
int publish_pose_for_viewing(float tx, float ty, float tz, float qx, float qy, float qz, float qw, ros::Publisher pose_marker_pub, std::string name, float r, float g, float b, float a, float size)
{
	visualization_msgs::Marker marker_pose;

	marker_pose.header.frame_id = "world";
	marker_pose.ns = name;
	marker_pose.type = visualization_msgs::Marker::SPHERE;
	marker_pose.action = visualization_msgs::Marker::ADD;
	marker_pose.scale.x = size;
	marker_pose.scale.y = size;
	marker_pose.scale.z = size;
	marker_pose.color.a = a;
	marker_pose.color.r = r;//float(rand()*256) / 255;
	marker_pose.color.g = g;//float(rand()*256) / 255;
	marker_pose.color.b = b;//float(rand()*256) / 255;
	marker_pose.lifetime = ros::Duration(10);


	marker_pose.header.stamp = ros::Time::now();
	marker_pose.id = 0;
	marker_pose.pose.position.x = tx;
	marker_pose.pose.position.y = ty;
	marker_pose.pose.position.z = tz;
	marker_pose.pose.orientation.x = qx;
	marker_pose.pose.orientation.y = qy;
	marker_pose.pose.orientation.z = qz;
	marker_pose.pose.orientation.w = qw;

  	pose_marker_pub.publish(marker_pose);

/*
	tf::Vector3 tran_input;
	tf::Quaternion quat_input((qx),qy,qz,qw);
	tran_input.setX(tx);
	tran_input.setY(ty);
	tran_input.setZ(tz);
	
	static tf::TransformBroadcaster br;

	tf::Transform transformToSend;
	transformToSend.setOrigin(tran_input);
	transformToSend.setRotation(quat_input);

	br.sendTransform(tf::StampedTransform(transformToSend, marker_pose.header.stamp, "world", name));*/

  	return 0;
}

/**
 * Builds a spheric marker for the specified pose to view the pose in rviz
 * @param marker_pose the marker is returned here
 * @param pose the pose to build the marker for
 * @param name the name for the marker
 * @param r the red component of the color of the marker
 * @param g the green component of the color of the marker
 * @param b the blue component of the color of the marker
 * @param a the alpha component of the color of the marker
 * @param size the sie of the sphere
 */
int buildMarker(visualization_msgs::Marker& marker_pose, const geometry_msgs::Pose& pose, std::string name, float r, float g, float b, float a, float size)
{
	marker_pose.header.frame_id = "world";
	marker_pose.ns = name;
	marker_pose.type = visualization_msgs::Marker::SPHERE;
	marker_pose.action = visualization_msgs::Marker::ADD;
	marker_pose.scale.x = size;
	marker_pose.scale.y = size;
	marker_pose.scale.z = size;
	marker_pose.color.a = a;
	marker_pose.color.r = r;//float(rand()*256) / 255;
	marker_pose.color.g = g;//float(rand()*256) / 255;
	marker_pose.color.b = b;//float(rand()*256) / 255;
	marker_pose.lifetime = ros::Duration(10);


	marker_pose.header.stamp = ros::Time::now();
	marker_pose.id = 0;
	marker_pose.pose.position.x = pose.position.x;
	marker_pose.pose.position.y = pose.position.y;
	marker_pose.pose.position.z = pose.position.z;
	marker_pose.pose.orientation.x = pose.orientation.x;
	marker_pose.pose.orientation.y = pose.orientation.y;
	marker_pose.pose.orientation.z = pose.orientation.z;
	marker_pose.pose.orientation.w = pose.orientation.w;

	return 0;
}


/**
 * Builds a spheric marker for the specified pose to view the pose in rviz
 * @param marker_pose the marker is returned here
 * @param position the position to put the marker at
 * @param name the name for the marker
 * @param r the red component of the color of the marker
 * @param g the green component of the color of the marker
 * @param b the blue component of the color of the marker
 * @param a the alpha component of the color of the marker
 * @param size the sie of the sphere
 */
int buildMarker(visualization_msgs::Marker& marker_pose, const cv::Point3f& position, std::string name, float r, float g, float b, float a, float size)
{
	marker_pose.header.frame_id = "world";
	marker_pose.ns = name;
	marker_pose.type = visualization_msgs::Marker::SPHERE;
	marker_pose.action = visualization_msgs::Marker::ADD;
	marker_pose.scale.x = size;
	marker_pose.scale.y = size;
	marker_pose.scale.z = size;
	marker_pose.color.a = a;
	marker_pose.color.r = r;//float(rand()*256) / 255;
	marker_pose.color.g = g;//float(rand()*256) / 255;
	marker_pose.color.b = b;//float(rand()*256) / 255;
	marker_pose.lifetime = ros::Duration(10);


	marker_pose.header.stamp = ros::Time::now();
	marker_pose.id = 0;
	marker_pose.pose.position.x = position.x;
	marker_pose.pose.position.y = position.y;
	marker_pose.pose.position.z = position.z;
	marker_pose.pose.orientation.x = 1;
	marker_pose.pose.orientation.y = 0;
	marker_pose.pose.orientation.z = 0;
	marker_pose.pose.orientation.w = 0;

	return 0;
}

/**
 * Computes the euclidean distance between the two provided poses
 */
double poseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	return std::sqrt((pose1.position.x*pose1.position.x - pose2.position.x*pose2.position.x) + 
					 (pose1.position.y*pose1.position.y - pose2.position.y*pose2.position.y) +
					 (pose1.position.z*pose1.position.z - pose2.position.z*pose2.position.z));
}

uint16_t getPixelSafe(const Mat& image, const Point2i p, uint16_t outOfBoundsValue)
{
	if(p.x<0 || p.y<0 || p.x>=image.cols || p.y>=image.rows)
		return outOfBoundsValue;
	else
		return image.at<uint16_t>(p);
}

Point2i findNearestNonZeroPixel(const Mat& image, int x, int y, double maxDist)
{
	//ROS_INFO("searching closest non-zero pixel for %d;%d",x,y);
	if(image.type()!=CV_16U)
	{
		throw std::invalid_argument("Only CV_16U mats are supported");
	}
	int bestx=x+maxDist+1;
	int besty=y+maxDist+1;
	double bestDist=maxDist+1;

	if(getPixelSafe(image,Point2i(x,y),0)!=0)
		return Point2i(x,y);

	auto checkPixel = [&](int xToCheck, int yToCheck)
	{
		//ROS_INFO("checking %d;%d",xToCheck,yToCheck);
		if(getPixelSafe(image,Point2i(xToCheck,yToCheck),0)!=0)
		{
			double dist = hypot(xToCheck-x,yToCheck-y);
			if(dist<bestDist)
			{
				bestDist = dist;
				bestx=xToCheck;
				besty=yToCheck;
				maxDist=bestDist;
			}
		}
	};
	for(int r = 1; r<=maxDist; r++)
	{
		checkPixel(x+r,y);
		checkPixel(x,y+r);
		checkPixel(x-r,y);
		checkPixel(x,y-r);
		for(int i=1;i<=r;i++)
		{
			checkPixel(x+r,y-i);
			checkPixel(x+r,y+i);

			checkPixel(x-r,y-i);
			checkPixel(x-r,y+i);

			checkPixel(x-i,y+r);
			checkPixel(x+i,y+r);

			checkPixel(x-i,y-r);
			checkPixel(x+i,y-r);
		}
	}

	if(bestDist==maxDist+1)
		return Point2i(x,y);
	return Point2i(bestx,besty);
}


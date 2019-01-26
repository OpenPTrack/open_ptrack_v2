
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
 * Converts a pose expressed as an opencv position vector and an opencv orientation vector
 * to a pose expressed as an Eigen vector and an Eigen quateronion
 * @param tvec opencv position vector
 * @param rvecV opencv rotation vector
 * @param Translate Eigen position vector
 * @param quats Eigen quaternion
 */
void opencvPoseToEigenPose(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Vector3d &translation, Eigen::Quaterniond &quaternion)
{

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d eigenMat;
    cv2eigen(R, eigenMat);
    quaternion = Eigen::Quaterniond(eigenMat);


    translation.x() = tvec[0];
    translation.y() = tvec[1];
    translation.z() = tvec[2];
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
visualization_msgs::Marker buildMarker(const geometry_msgs::Pose& pose, std::string name, float r, float g, float b, float a, float size, std::string frame_id)
{
	visualization_msgs::Marker marker_pose;
	marker_pose.header.frame_id = frame_id;
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

	return marker_pose;
}


/**
 * Builds a spheric marker for the specified pose to view the pose in rviz
 * @param position the position to put the marker at
 * @param name the name for the marker
 * @param r the red component of the color of the marker
 * @param g the green component of the color of the marker
 * @param b the blue component of the color of the marker
 * @param a the alpha component of the color of the marker
 * @param size the sie of the sphere
 */
visualization_msgs::Marker buildMarker(const cv::Point3f& position, std::string name, float r, float g, float b, float a, float size, std::string frame_id)
{
	visualization_msgs::Marker marker_pose;
	marker_pose.header.frame_id = frame_id;
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

	return marker_pose;
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





void transformCvPoint3f(const cv::Point3f& in, cv::Point3f& out, tf::StampedTransform transform)
{
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = in.x;
	pose.pose.position.y = in.y;
	pose.pose.position.z = in.z;

	pose.pose.orientation.x = 1;
	pose.pose.orientation.y = 0;
	pose.pose.orientation.z = 0;
	pose.pose.orientation.w = 0;

	geometry_msgs::TransformStamped transformMsg;
	//tf2::convert(transformKinectToWorld, transformMsg);
	tf::transformStampedTFToMsg(transform,transformMsg);

	tf2::doTransform(pose,pose,transformMsg);

	out.x = pose.pose.position.x;
	out.y = pose.pose.position.y;
	out.z = pose.pose.position.z;

}

/** Prepares an opencv image to be shown in a new window. To actually display it you will have to call cv::waitKey(int)
 *
 * @param winName The name to give to the window
 * @param image The image to be shown
 * @param winHeight The height the displayed image will have in the screen, in pixels
 * @param winHeight The width the displayed image will have in the screen, in pixels. If it is -1 it will be deduced from winHeight using the aspect of the image
 *
 */
void prepareOpencvImageForShowing(std::string winName, cv::Mat image, int winHeight, int winWidth=-1)
{
	cv::namedWindow(winName, cv::WINDOW_NORMAL);
	if(winWidth==-1)
		winWidth=(int)(((double)winHeight)/image.rows*image.cols);
	cv::resizeWindow(winName,winWidth,winHeight);
    cv::imshow(winName,image);
}


/**
 *	Publishes the provided pose as a tf frame
 *	
 *	@param pose The pose to be published
 *	@param tfFrameName The name the of the tf frame the pose will be published as
 */
void publishPoseAsTfFrame(const geometry_msgs::PoseStamped& pose, std::string tfFrameName)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
	tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, pose.header.frame_id, tfFrameName));
}

/**
 *	Publishes the provided transform as a tf frame
 *	
 *	@param pose The pose to be published
 *	@param tfFrameName The name the of the tf frame the pose will be published as
 */
void publishTransformAsTfFrame(const tf::Transform& transform, std::string tfFrameName, std::string parentFrame, const ros::Time& time)
{
	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, time, parentFrame, tfFrameName));
}


geometry_msgs::Point buildRosPoint(double positionX, double positionY, double positionZ)
{
	geometry_msgs::Point point;
	point.x=positionX;
	point.y=positionY;
	point.z=positionZ;
	return point;
}

geometry_msgs::Quaternion buildRosQuaternion(double quaternionX, double quaternionY, double quaternionZ, double quaternionW)
{
	geometry_msgs::Quaternion quaternion;
	quaternion.x = quaternionX;
	quaternion.y = quaternionY;
	quaternion.z = quaternionZ;
	quaternion.w = quaternionW;
	return quaternion;
}


geometry_msgs::Pose buildRosPose(double positionX, double positionY, double positionZ, double quaternionX, double quaternionY, double quaternionZ, double quaternionW)
{
	geometry_msgs::Pose pose;//the compiler should optimize this and don't do a copy on return (guaranteed in c++17)
	pose.position.x = positionX;
	pose.position.y = positionY;
	pose.position.z = positionZ;

	pose.orientation.x = quaternionX;
	pose.orientation.y = quaternionY;
	pose.orientation.z = quaternionZ;
	pose.orientation.w = quaternionW;
	return pose;
}

geometry_msgs::Pose buildRosPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
	return buildRosPose(position.x(),position.y(),position.z(),orientation.x(),orientation.y(),orientation.z(),orientation.w());
}

std::string poseToString(tf::Pose pose)
{
	return ""+to_string(pose.getOrigin().getX())+";"+to_string(pose.getOrigin().getY())+";"+to_string(pose.getOrigin().getZ())+"      "+to_string(pose.getRotation().getX())+";"+to_string(pose.getRotation().getY())+";"+to_string(pose.getRotation().getZ())+";"+to_string(pose.getRotation().getW());
}



/**
 * Converts a transform in a left-handed coordinate space to a transform in a righ-handed coordinate space
 * Note that tf::Pose and tf::Transform are the same thing
 */
tf::Transform leftHandedToRightHanded(const tf::Transform& leftHandedPose)
{
	tf::Vector3 leftHandedOrigin = leftHandedPose.getOrigin();
	tf::Vector3 rightHandedOrigin = tf::Vector3(leftHandedOrigin.getX(),-leftHandedOrigin.getY(),leftHandedOrigin.getZ());

	tf::Vector3 leftHandedRotationAxis = leftHandedPose.getRotation().getAxis();
	tfScalar leftHandedRotationAngle = leftHandedPose.getRotation().getAngle();
	tf::Quaternion rightHandedRotation = tf::Quaternion(tf::Vector3(leftHandedRotationAxis.getX(),-leftHandedRotationAxis.getY(),leftHandedRotationAxis.getZ()),-leftHandedRotationAngle);

	return tf::Transform(rightHandedRotation,rightHandedOrigin);
}
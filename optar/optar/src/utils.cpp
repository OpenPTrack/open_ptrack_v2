/**
 * @file
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 * Collection of utility methods used by the optar module
 */

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
#include <tf2_ros/static_transform_broadcaster.h>

#include "utils.hpp"

using namespace std;
using namespace cv;


/**
 * Calculates the 3d coordinates of a point using the image pixel coordinates and the depth
 * @param[in]  x               x pixel position
 * @param[in]  y               y pixel position
 * @param[in]  depth_mm        depth of the pixel in millimiters
 * @param[in]  focalLengthX    x component of the focal length of the camera
 * @param[in]  focalLengthY    y component of the focal length of the camera
 * @param[in]  principalPointX x component of the principal  point of the camera
 * @param[in]  principalPointY y component of the principal  point of the camera
 * @return                 The 3d position
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
 * Converts a pose expressed wih the OpenCV translation-rotationvector convention
 * to a pose expressed as an Eigen vector and an Eigen quaternion
 *
 * @param[in] rvec        OpenCV orientation vector
 * @param[in] tvec        OpenCV translation vector
 * @param     translation output Eigen translation vector
 * @param     quaternion  output Eigen quaternion
 */
void opencvPoseToEigenPose(const cv::Vec3d& rvec, const cv::Vec3d& tvec, Eigen::Vector3d &translation, Eigen::Quaterniond &quaternion)
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
 * Convert a tf pose to the opencv translation-rotationvector representation
 *
 * @param[in]  pose  The tf pose
 * @param      rvec  The rotation vector
 * @param      tvec  The translation vector
 */
void tfPoseToOpenCvPose(const tf::Pose& pose, cv::Vec3d& rvec, cv::Vec3d& tvec)
{
	tf::Quaternion tfQuat = pose.getRotation();
	Eigen::Quaterniond eigenQuat(tfQuat.getW(),tfQuat.getX(),tfQuat.getY(),tfQuat.getZ());
    cv::Mat cvRotMat;
    eigen2cv(eigenQuat.toRotationMatrix() ,cvRotMat);
    cv::Mat cvRvec;
    cv::Rodrigues(cvRotMat, cvRvec);

    tvec[0] = pose.getOrigin().x();
    tvec[1] = pose.getOrigin().y();
    tvec[2] = pose.getOrigin().z();
}

/**
 * Publishes a pose as a sphere on a MarkerArray topic
 * @param  tx              x position of the pose
 * @param  ty              y position of the pose
 * @param  tz              z position of the pose
 * @param  qx              x component of the orientation quaternion
 * @param  qy              y component of the orientation quaternion
 * @param  qz              z component of the orientation quaternion
 * @param  qw              w component of the orientation quaternion
 * @param  pose_marker_pub publisher to use toto publish the marker
 * @param  name            name for the marker
 * @param  r               red component of the color of the marker
 * @param  g               green component of the color of the marker
 * @param  b               blue component of the color of the marker
 * @param  a               alpha component of the color of the marker
 * @param  size            diameter of the spehere
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
 * Builds an speric marker for the specified pose. By publishing the produced marker you can view the pose in rviz
 *
 * @param x 		the x position to put the marker at
 * @param y 		the y position to put the marker at
 * @param z 		the z position to put the marker at
 * @param name 		the name for the marker
 * @param r 		the red component of the color of the marker
 * @param g 		the green component of the color of the marker
 * @param b 		the blue component of the color of the marker
 * @param a 		the alpha component of the color of the marker
 * @param size 		the sie of the sphere
 * @param frame_id	the tf frame to set in the marker header
 *
 * @return The marker
 */
visualization_msgs::Marker buildMarker(float x, float y, float z, std::string name, float r, float g, float b, float a, float size, std::string frame_id)
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
	marker_pose.pose.position.x = x;
	marker_pose.pose.position.y = y;
	marker_pose.pose.position.z = z;
	marker_pose.pose.orientation.x = 1;
	marker_pose.pose.orientation.y = 0;
	marker_pose.pose.orientation.z = 0;
	marker_pose.pose.orientation.w = 0;

	return marker_pose;
}




/**
 * Builds an arrow marker for the specified pose. By publishing the produced marker you can view the pose in rviz
 *
 * @param x 		the x position to put the marker at
 * @param y 		the y position to put the marker at
 * @param z 		the z position to put the marker at
 * @param name 		the name for the marker
 * @param r 		the red component of the color of the marker
 * @param g 		the green component of the color of the marker
 * @param b 		the blue component of the color of the marker
 * @param a 		the alpha component of the color of the marker
 * @param size 		the sie of the sphere
 * @param frame_id	the tf frame to set in the marker header
 * @param orient_x 	quaternion orientation x
 * @param orient_y 	quaternion orientation y
 * @param orient_z 	quaternion orientation z
 * @param orient_w 	quaternion orientation w
 *
 * @return The built marker
 */
visualization_msgs::Marker buildArrowMarker(float x, float y, float z, std::string name, float r, float g, float b, float a, float size, std::string frame_id,float orient_x,float orient_y,float orient_z,float orient_w)
{
	visualization_msgs::Marker marker_pose;
	marker_pose.header.frame_id = frame_id;
	marker_pose.ns = name;
	marker_pose.type = visualization_msgs::Marker::ARROW;
	marker_pose.action = visualization_msgs::Marker::ADD;
	marker_pose.scale.x = size;
	marker_pose.scale.y = size/5;
	marker_pose.scale.z = size/5;
	marker_pose.color.a = a;
	marker_pose.color.r = r;//float(rand()*256) / 255;
	marker_pose.color.g = g;//float(rand()*256) / 255;
	marker_pose.color.b = b;//float(rand()*256) / 255;
	marker_pose.lifetime = ros::Duration(10);


	marker_pose.header.stamp = ros::Time::now();
	marker_pose.id = 0;
	marker_pose.pose.position.x = x;
	marker_pose.pose.position.y = y;
	marker_pose.pose.position.z = z;
	marker_pose.pose.orientation.x = orient_x;
	marker_pose.pose.orientation.y = orient_y;
	marker_pose.pose.orientation.z = orient_z;
	marker_pose.pose.orientation.w = orient_w;

	return marker_pose;
}

/**
 * Builds a marker that deletes an already published marker from rviz
 *
 * @param name the name for the marker to be deleted
 *
 * @return The built marker
 */
visualization_msgs::Marker buildDeletingMarker(std::string name)
{
	visualization_msgs::Marker marker_pose;
	marker_pose.ns = name;
	marker_pose.action = visualization_msgs::Marker::DELETE;

	return marker_pose;
}



/**
 * Builds an speric marker for the specified pose. By publishing the produced marker you can view the pose in rviz
 *
 * @param pose 		the pose to build the marker for
 * @param name 		the name for the marker
 * @param r 		the red component of the color of the marker
 * @param g 		the green component of the color of the marker
 * @param b 		the blue component of the color of the marker
 * @param a 		the alpha component of the color of the marker
 * @param size 		the sie of the sphere
 * @param frame_id	the tf frame to set in the marker header
 *
 * @return The built marker
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
 * Builds an speric marker for the specified pose. By publishing the produced marker you can view the pose in rviz
 *
 * @param position the position to put the marker at
 * @param name the name for the marker
 * @param r the red component of the color of the marker
 * @param g the green component of the color of the marker
 * @param b the blue component of the color of the marker
 * @param a the alpha component of the color of the marker
 * @param size the sie of the sphere
 * @param frame_id	the tf frame to set in the marker header
 *
 * @return The built marker
 */
visualization_msgs::Marker buildMarker(const cv::Point3f& position, std::string name, float r, float g, float b, float a, float size, std::string frame_id)
{
	return buildMarker(position.x, position.y, position.z, name, r, g, b, a, size,  frame_id);
}


/**
 * Computes the euclidean distance between the two provided poses
 *
 * @param[in]  pose1  The pose 1
 * @param[in]  pose2  The pose 2
 *
 * @return The euclidean distance
 */
double poseDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{

	return std::sqrt((pose1.position.x - pose2.position.x)*(pose1.position.x - pose2.position.x) +
									 (pose1.position.y - pose2.position.y)*(pose1.position.y - pose2.position.y) +
									 (pose1.position.z - pose2.position.z)*(pose1.position.z - pose2.position.z));
}

/**
 * Computes the euclidean distance between the two provided poses
 *
 * @param[in]  pose1  The pose 1
 * @param[in]  pose2  The pose 2
 *
 * @return The euclidean distance
 */
double poseDistance(tf::Pose pose1, tf::Pose pose2)
{
	tf::Vector3 p1 = pose1.getOrigin();
	tf::Vector3 p2 = pose2.getOrigin();
	return std::sqrt(	(p1.x()-p2.x())*(p1.x()-p2.x()) +
										(p1.y()-p2.y())*(p1.y()-p2.y()) +
										(p1.z()-p2.z())*(p1.z()-p2.z()));
}


/**
 * Gets the specified pixel from the image, and if the specified coordinetes are outside the bounds of the
 * image, it returns the provided value.
 *
 * @param[in]  image             The image
 * @param[in]  p                 The requested pixel's position
 * @param[in]  outOfBoundsValue  The value returned if p is out of the image bounds
 *
 * @return     The pixel value or the value in outOfBoundsValue
 */
uint16_t getPixelSafe(const Mat& image, const Point2i p, uint16_t outOfBoundsValue)
{
	if(p.x<0 || p.y<0 || p.x>=image.cols || p.y>=image.rows)
		return outOfBoundsValue;
	else
		return image.at<uint16_t>(p);
}




/**
 * Finds the closest non-zero pixel for the specified position in the specified image
 *
 * @param[in]  image    The image
 * @param[in]  x        The x position
 * @param[in]  y        The y position
 * @param[in]  maxDist  The maximum allowed distance
 *
 * @return The closest non-zero pixel position. If no non-zero pixel has been found within the specified area
 *         it will return a pixel that is not in the specified area
 */
cv::Point2i findNearestNonZeroPixel(const cv::Mat& image, int x, int y, double maxDist)
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
			if(bestDist<=maxDist)//if you did find it, exit
				break;
			checkPixel(x+r,y-i);
			checkPixel(x+r,y+i);

			checkPixel(x-r,y-i);
			checkPixel(x-r,y+i);

			checkPixel(x-i,y+r);
			checkPixel(x+i,y+r);

			checkPixel(x-i,y-r);
			checkPixel(x+i,y-r);
		}
		if(bestDist<=maxDist)//if you did find it, exit
			break;
	}

	if(bestDist==maxDist+1)
		return Point2i(x,y);
	return Point2i(bestx,besty);
}

/**
 * Finds the non-zero pixel with the lowest value in a ring centered around a specific pixel
 * @param[in] image the image in which to search
 * @param     x the x coordinate of the center of the ring
 * @param     y the y coordinate of the center of the ring
 * @param     maxRadius the outer radius of the ring
 * @param     minRadius the inner radius of the ring
 *
 * @return the pixel, if no non-zero pixel is found it returns [x,y], i.e. the central pixel
 */
cv::Point2i findLowestNonZeroInRing(const cv::Mat& image, int x, int y, double maxRadius, double minRadius)
{
	//ROS_INFO("searching closest non-zero pixel for %d;%d",x,y);
	if(image.type()!=CV_16U)
	{
		throw std::invalid_argument("Only CV_16U mats are supported");
	}
	int bestx=x+maxRadius+1;
	int besty=y+maxRadius+1;
	int bestValue = INT_MAX;

	//function that checks if the pixel is in the required area and is the best one
	//if it is, it sets the bextx, bety, bestValue variables accordingly
	auto checkPixel = [&](int xToCheck, int yToCheck)
	{
		//ROS_INFO("checking %d;%d",xToCheck,yToCheck);
		int value = getPixelSafe(image,Point2i(xToCheck,yToCheck),0);
		if(value!=0)
		{
			double dist = hypot(xToCheck-x,yToCheck-y);
			//if it's in the ring and it's the best until now
			if(value<bestValue && minRadius<=dist && dist<=maxRadius)
			{
				bestx = xToCheck;
				besty = yToCheck;
				bestValue = value;
			}
		}
	};

	if(minRadius==0 && getPixelSafe(image,Point2i(x,y),0)!=0)
	{
		checkPixel(x,y);
	}

	//loop through all the values in the square ring that contains our ring
	int squareMaxRadius = maxRadius;
	int squareMinRadius = minRadius/std::sqrt(2);
	for(int r = squareMinRadius; r<=squareMaxRadius; r++)
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

	if(bestValue==INT_MAX)//if we didn't find anything
		return Point2i(x,y);
	return Point2i(bestx,besty);
}



/**
 * @brief      Transforms the provided position using the provided tf transform
 *
 * @param[in]  in         The input point
 * @param      out        The output point
 * @param[in]  transform  The transform to use
 */
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

/** Prepares an opencv image to be shown in a new window. To actually display it you will have
 *  to call cv::waitKey(int)
 *
 * @param winName    The name to give to the window
 * @param image      The image to be shown
 * @param winHeight  The height the displayed image will have in the screen, in pixels
 * @param winWidth   The width the displayed image will have in the screen, in pixels. If it is -1
 *                   it will be deduced from winHeight using the aspect of the image
 *
 */
void prepareOpencvImageForShowing(std::string winName, cv::Mat image, int winHeight, int winWidth)
{
	cv::namedWindow(winName, cv::WINDOW_NORMAL);
	if(winWidth==-1)
		winWidth=(int)(((double)winHeight)/image.rows*image.cols);
	cv::resizeWindow(winName,winWidth,winHeight);
    cv::imshow(winName,image);
}

/**
 *
 * Publishes the provided transform as a tf frame
 *
 * @param stampedTransform   The transform to be published
 */
void publishTransformAsTfFrame(const tf::StampedTransform& stampedTransform)
{
	static tf2_ros::StaticTransformBroadcaster br;
	geometry_msgs::TransformStamped stampedTransformMsg;
	tf::transformStampedTFToMsg(stampedTransform,stampedTransformMsg);
	br.sendTransform(stampedTransformMsg);
}

/**
 *
 * Publishes the provided transform as a tf frame
 *
 * @param transform   The transform to be published
 * @param tfFrameName The name to use for the tf frame
 * @param parentFrame The name of the parent frame
 * @param time        The time to set on the published frame transform
 */
void publishTransformAsTfFrame(const tf::Transform& transform, std::string tfFrameName, std::string parentFrame, const ros::Time& time)
{
	publishTransformAsTfFrame(tf::StampedTransform(transform, time, parentFrame, tfFrameName));
}

/**
 *	Publishes the provided pose as a tf frame. It uses the timestamp in the pose header
 *	as the timestamp for the tf frame transform
 *
 *	@param pose The pose to be published
 *	@param tfFrameName The name of the tf frame the pose will be published as
 */
void publishPoseAsTfFrame(const geometry_msgs::PoseStamped& pose, std::string tfFrameName)
{
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
	tf::Quaternion q(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
	transform.setRotation(q);
	publishTransformAsTfFrame(transform,tfFrameName,pose.header.frame_id,pose.header.stamp);
}

/**
 * @brief      Builds a geometry_msgs::Point object from doubles
 *
 * @param[in]  positionX  The x coordinate
 * @param[in]  positionY  The y coordinate
 * @param[in]  positionZ  The z coordinate
 *
 * @return     The geometry_msgs::Point object
 */
geometry_msgs::Point buildRosPoint(double positionX, double positionY, double positionZ)
{
	geometry_msgs::Point point;
	point.x=positionX;
	point.y=positionY;
	point.z=positionZ;
	return point;
}


/**
 * @brief      Builds a geometry_msgs::Quaternion object from doubles
 *
 * @param[in]  quaternionX  The quaternion x component
 * @param[in]  quaternionY  The quaternion y component
 * @param[in]  quaternionZ  The quaternion z component
 * @param[in]  quaternionW  The quaternion w component
 *
 * @return     The geometry_msgs::Quaternion object
 */
geometry_msgs::Quaternion buildRosQuaternion(double quaternionX, double quaternionY, double quaternionZ, double quaternionW)
{
	geometry_msgs::Quaternion quaternion;
	quaternion.x = quaternionX;
	quaternion.y = quaternionY;
	quaternion.z = quaternionZ;
	quaternion.w = quaternionW;
	return quaternion;
}

/**
 * @brief      Builds a geometry_msgs::Pose object from doubles
 *
 * @param[in]  positionX    The position x coordinate
 * @param[in]  positionY    The position y coordinate
 * @param[in]  positionZ    The position z coordinate
 * @param[in]  quaternionX  The quaternion x component
 * @param[in]  quaternionY  The quaternion y component
 * @param[in]  quaternionZ  The quaternion z component
 * @param[in]  quaternionW  The quaternion w component
 *
 * @return     The geometry_msgs::Pose object
 */
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

/**
 * @brief      Builds a geometry_msgs::Pose object from position and orientation
 *
 * @param[in]  position     The position
 * @param[in]  orientation  The orientation
 *
 * @return     The geometry_msgs::Pose object
 */
geometry_msgs::Pose buildRosPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
{
	return buildRosPose(position.x(),position.y(),position.z(),orientation.x(),orientation.y(),orientation.z(),orientation.w());
}

/**
 * Provides a human-readable string representation of a pose
 *
 * @return The string
 */
std::string poseToString(tf::Pose pose)
{
	return ""+to_string(pose.getOrigin().getX())+";"+to_string(pose.getOrigin().getY())+";"+to_string(pose.getOrigin().getZ())+";"+to_string(pose.getRotation().getX())+";"+to_string(pose.getRotation().getY())+";"+to_string(pose.getRotation().getZ())+";"+to_string(pose.getRotation().getW());
}


/**
 * Provides a human-readable string representation of a pose
 *
 * @return The string
 */
std::string poseToString(geometry_msgs::Pose pose)
{
	tf::Pose poseTf;
	poseMsgToTF(pose,poseTf);
	return poseToString(poseTf);
}

/**
 * Converts a transform in a left-handed coordinate space to a transform in a right-handed coordinate space
 *
 * @param[in]  leftHandedPose  The left handed pose (i.e. transform)
 *
 * @return      The right handed pose
 */
tf::Transform convertPoseUnityToRos(const tf::Transform& leftHandedPose)
{
	tf::Vector3 leftHandedOrigin = leftHandedPose.getOrigin();
	tf::Vector3 rightHandedOrigin = tf::Vector3(leftHandedOrigin.getX(),-leftHandedOrigin.getY(),leftHandedOrigin.getZ());

	tf::Vector3 leftHandedRotationAxis = leftHandedPose.getRotation().getAxis();
	tfScalar leftHandedRotationAngle = leftHandedPose.getRotation().getAngle();
	tf::Quaternion rightHandedRotation = tf::Quaternion(tf::Vector3(leftHandedRotationAxis.getX(),-leftHandedRotationAxis.getY(),leftHandedRotationAxis.getZ()),-leftHandedRotationAngle);

	return tf::Transform(rightHandedRotation,rightHandedOrigin);
}


/**
 * Computes the average position of the provided poses (does not look at the orientations)
 *
 * @param poses The poses to average
 *
 * @return The average position
 */
tf::Vector3 averagePosePositions(const std::vector<tf::Pose> poses)
{
	double x;
	double y;
	double z;
	for(tf::Pose pose : poses)
	{
		tf::Vector3 position = pose.getOrigin();
		x += position.getX();
		y += position.getY();
		z += position.getZ();
	}
	x/=poses.size();
	y/=poses.size();
	z/=poses.size();
	return tf::Vector3(x,y,z);
}


/**
 * Checks if the pose is valid, i.e. if it contains nan values or infinite values and if the quaternion is normalized
 *
 * @param[in] pose The pose to be checked
 *
 * @return True if valid, false if invalid
 */
bool isPoseValid(const tf::Pose& pose)
{
	tf::Vector3 origin = pose.getOrigin();
	if(std::isnan(origin.getX()) || std::isnan(origin.getY()) || std::isnan(origin.getZ()))
		return false;
	if(std::isinf(origin.getX()) || std::isinf(origin.getY()) || std::isinf(origin.getZ()))
		return false;

	tf::Quaternion orientation = pose.getRotation();
	if(std::isnan(orientation.getX()) || std::isnan(orientation.getY()) || std::isnan(orientation.getZ()) || std::isnan(orientation.getW()))
		return false;
	if(std::isinf(orientation.getX()) || std::isinf(orientation.getY()) || std::isinf(orientation.getZ()) || std::isinf(orientation.getW()))
		return false;


	bool isNormalized = std::abs((orientation.w() * orientation.w()
                        + orientation.x() * orientation.x()
                        + orientation.y() * orientation.y()
                        + orientation.z() * orientation.z()) - 1.0f) < 10e-6;//is this a sensible threshold?
	if(!isNormalized)
		return false;

	return true;
}


/**
 * Builds a PoseStamepd from a regular Pose
 * @param  pose      The pose to use
 * @param  frame_id  The frame id to assign to the pose
 * @param  timestamp The timestamp for the pose
 * @return           The resolution stamped pose
 */
geometry_msgs::PoseStamped poseToPoseStamped(const geometry_msgs::Pose& pose, std::string frame_id, ros::Time timestamp)
{
	geometry_msgs::PoseStamped poseStamped;
	poseStamped.pose = pose;
	poseStamped.header.frame_id = frame_id;
	poseStamped.header.stamp = timestamp;

	return poseStamped;
}



/**
 * Converts the mobile camera pose from the ARCore convention to the ROS covention
 * ARCore on Unity uses Unity's coordinate system, which is left-handed, normally in arcore
 * for Android the arcore camera position is defined with x pointing right, y pointing up and
 * -z pointing where the camera is facing.
 * ROS uses a right-handed system, with x pointing right, y pointing down and z pointing where
 * the camera is facing.
 * As provided from all ARCore APIs, Poses always describe the transformation from object's
 * local coordinate space to the world coordinate space. This is the usual pose representation,
 * same as ROS.
 * @param  cameraPoseArcore The camera pose in ARCore's convention
 * @return                  The camera pose in the ROS convention
 */
tf::Pose convertCameraPoseArcoreToRos(const geometry_msgs::Pose& cameraPoseArcore)
{
		tf::Pose phonePoseArcoreFrameUnity;
		tf::poseMsgToTF(cameraPoseArcore,phonePoseArcoreFrameUnity);
		return convertCameraPoseArcoreToRos(phonePoseArcoreFrameUnity);
}


/**
 * Converts the mobile camera pose from the ARCore convention to the ROS covention
 * ARCore on Unity uses Unity's coordinate system, which is left-handed, normally in arcore
 * for Android the arcore camera position is defined with x pointing right, y pointing up and
 * -z pointing where the camera is facing.
 * ROS uses a right-handed system, with x pointing right, y pointing down and z pointing where
 * the camera is facing.
 * As provided from all ARCore APIs, Poses always describe the transformation from object's
 * local coordinate space to the world coordinate space. This is the usual pose representation,
 * same as ROS.
 * @param  cameraPoseArcore The camera pose in ARCore's convention
 * @return                  The camera pose in the ROS convention
 */
tf::Pose convertCameraPoseArcoreToRos(const tf::Pose& cameraPoseArcore)
{

		// Convert phone arcore pose
		// ARCore on Unity uses Unity's coordinate systema, which is left-handed, normally in arcore for Android the arcore
		// camera position is defined with x pointing right, y pointing up and -z pointing where the camera is facing.
		// As provided from all ARCore APIs, Poses always describe the transformation from object's local coordinate space
		// to the world coordinate space. This is the usual pose representation, same as ROS
		tf::Pose phonePoseArcoreFrameUnity = cameraPoseArcore;
		tf::Pose phonePoseArcoreFrame = convertPoseUnityToRos(phonePoseArcoreFrameUnity);

		//publishTransformAsTfFrame(phonePoseArcoreFrame,"phone_arcore","/world",arcoreInputMsg->header.stamp);
		//publishTransformAsTfFrame(phonePoseArcoreFrameUnity,"phone_arcore_left","/world",arcoreInputMsg->header.stamp);

		//from x to the right, y up, z back to x to the right, y down, z forward
		tf::Transform cameraConventionTransform = tf::Transform(tf::Quaternion(tf::Vector3(1,0,0), 3.1415926535897));//rotate 180 degrees around x axis
		//assuming z is pointing foward:
		tf::Transform portraitToLandscape = tf::Transform(tf::Quaternion(tf::Vector3(0,0,1), 3.1415926535897/2));//rotate +90 degrees around z axis
		tf::Transform justRotation = tf::Transform(phonePoseArcoreFrame.getRotation()) * portraitToLandscape;
		tf::Transform justTranslation = tf::Transform(tf::Quaternion(1,0,0,0),phonePoseArcoreFrame.getOrigin());

		//tf::Pose phonePoseArcoreInverted = tf::Transform(tf::Quaternion(tf::Vector3(1,0,0),0),phonePoseArcoreFrame.getOrigin()).inverse() * tf::Transform(phonePoseArcoreFrame.getRotation()).inverse();
		return  justTranslation *cameraConventionTransform*justRotation;
}



/**
 * Inverts a pose
 * @param  pose Input pose to be inverted
 * @return      The inverted pose
 */
geometry_msgs::Pose invertPose(const geometry_msgs::Pose& pose)
{
	tf::Pose poseTf;
	tf::poseMsgToTF(pose,poseTf);
	geometry_msgs::Pose ret;
	tf::poseTFToMsg(poseTf.inverse(),ret);
	return ret;
}

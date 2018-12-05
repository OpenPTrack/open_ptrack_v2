#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "arcore_camera_image_republisher"

void imageCallback(const opt_msgs::ArcoreCameraImageConstPtr& inImg)
{
	ROS_INFO("received image");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::Subscriber sub = nh.subscribe("arcoreCameraImage", 1, imageCallback);

	ros::spin();
}



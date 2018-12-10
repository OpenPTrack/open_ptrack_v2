#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "arcore_camera_image_republisher"

ros::Publisher pub;

void imageCallback(const opt_msgs::ArcoreCameraImageConstPtr& inImg)
{
	ROS_INFO("received image");
	pub.publish(inImg->image);
	ROS_INFO("published image");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::Subscriber sub = nh.subscribe("/optar/arcore_camera", 1, imageCallback);
	pub = nh.advertise<sensor_msgs::CompressedImage>("/optar/camera/compressed", 10);
	ros::spin();
}



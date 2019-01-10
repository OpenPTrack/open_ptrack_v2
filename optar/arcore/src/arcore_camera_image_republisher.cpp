#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <cv_bridge/cv_bridge.h>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "arcore_camera_image_republisher"

ros::Publisher pub;

void imageCallback(const opt_msgs::ArcoreCameraImageConstPtr& inImg)
{
	ROS_INFO("received image");

	cv::Mat img = cv_bridge::toCvCopy(inImg->image)->image;//convert compressed image data to cv::Mat
	if(!img.data)
	{
		ROS_ERROR("couldn't extract kinect camera opencv image");
		return;
	}
    ROS_INFO("decoded kinect camera image");

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();

	pub.publish(msg);
	ROS_INFO("published image");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::Subscriber sub = nh.subscribe("/optar/arcore_camera", 1, imageCallback);
	pub = nh.advertise<sensor_msgs::CompressedImage>("/optar/camera_republished_raw", 10);
	ros::spin();
}



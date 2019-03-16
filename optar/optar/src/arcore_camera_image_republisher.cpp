#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <opt_msgs/ArcoreCameraFeatures.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "arcore_camera_image_republisher"

image_transport::Publisher pub;

void imageCallback(const opt_msgs::ArcoreCameraImageConstPtr& inImg)
{
	ROS_INFO("received image");

	cv::Mat img = cv_bridge::toCvCopy(inImg->image)->image;//convert compressed image data to cv::Mat
	if(!img.data)
	{
		ROS_ERROR("couldn't extract kinect camera opencv image");
		return;
	}
    ROS_INFO("decoded kinect camera image, it's %dx%d",img.cols,img.rows);

	//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
	if(img.channels()==3)
	{
		//take red channel
		cv::Mat planes[3];
		split(img,planes);  // planes[2] is the red channel
		img = planes[2];
		cv::Mat flippedArcoreImg;
		cv::flip(img,flippedArcoreImg,0);
		img=flippedArcoreImg;
	}
	else if(img.channels()==1)
	{
		cv::Mat flippedArcoreImg;
		cv::flip(img,flippedArcoreImg,0);
		img=flippedArcoreImg;
	}
	else
	{
		ROS_ERROR("received an invalid image, should have either one or three channels");
		return;
	}

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
	pub.publish(msg);




	ROS_INFO("published image");
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::Subscriber sub = nh.subscribe("input_topic", 1, imageCallback);
	ROS_INFO_STREAM("Subscribed to topic "<<ros::names::remap("input_topic"));
	image_transport::ImageTransport it(nh);
	pub = it.advertise("optar/camera_republished", 1);
	ros::spin();
}



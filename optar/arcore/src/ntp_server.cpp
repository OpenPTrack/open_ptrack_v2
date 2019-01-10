#include <ros/ros.h>
#include <opt_msgs/OptarNtpMessage.h>
#include <cv_bridge/cv_bridge.h>


ros::Publisher pub;

void callback(const opt_msgs::OptarNtpMessagePtr& inMsg)
{
	if(inMsg->type==opt_msgs::OptarNtpMessage::QUERY)
	{
		opt_msgs::OptarNtpMessage response;
		response.type = opt_msgs::OptarNtpMessage::REPLY;
		response.serverTime = ros::Time::now();
		response.id = inMsg->id;
		pub.publish(response);
		ROS_INFO("handled query");
	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ntp_server");
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting ntp_server");
	ros::Subscriber sub = nh.subscribe("/optar/ntp_chat", 1, callback);
	pub = nh.advertise<opt_msgs::OptarNtpMessage>("/optar/ntp_chat", 10);
	ros::spin();
}



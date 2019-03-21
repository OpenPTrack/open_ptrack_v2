/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * This file implements a ros node which estimates the transformation between an ARCore 
 * coordinae frame and the ros tf /world frame
 */


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opt_msgs/ARDevicePose.h>
#include <opt_msgs/ARDevicePoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <functional>
#include <visualization_msgs/MarkerArray.h>

#include "../utils.hpp"


using namespace std;


const string NODE_NAME 								= "ardevice_poses_republisher";
const string ardevice_poses_output_topic			= "ardevice_poses_output";
const string ardevice_poses_output_markers_topic	= "ardevice_poses_output_markers";
const string devices_heartbeats_topicName			= "heartbeats_topic" ;


static double no_update_timeout_millis = 2000;

class PoseReceiver
{
private:
	ros::Subscriber subscriber;
	shared_ptr<map<string, shared_ptr<PoseReceiver>>> handlers;
	shared_ptr<mutex> handlersMutex;
public:

	string deviceId;
	std::chrono::steady_clock::time_point lastMessageTime;
	geometry_msgs::PoseStampedConstPtr lastMessage;

	void poseCallback(const geometry_msgs::PoseStampedConstPtr poseMsg)
	{
		//ROS_INFO_STREAM(""<<deviceId<<": received ");
    	std::lock_guard<std::mutex> lock(*handlersMutex);
		lastMessageTime = std::chrono::steady_clock::now();	
    	lastMessage = poseMsg;
	}

	PoseReceiver(string deviceId,
				std::shared_ptr<ros::NodeHandle> nodeHandle,
				shared_ptr<map<string, shared_ptr<PoseReceiver>>> handlers,
				shared_ptr<mutex> handlersMutex)
	{
		this->deviceId = deviceId;
		this->handlers = handlers;
		this->handlersMutex = handlersMutex;

		string poseTopic = "/optar/"+deviceId+"/pose";
		subscriber = nodeHandle->subscribe(poseTopic, 10, &PoseReceiver::poseCallback, this);
		lastMessageTime = std::chrono::steady_clock::now();	
	}
};

shared_ptr<map<string, shared_ptr<PoseReceiver>>> handlers;
shared_ptr<mutex> handlersMutex;
shared_ptr<ros::NodeHandle> nodeHandle;
ros::Publisher	pose_marker_pub;


shared_ptr<opt_msgs::ARDevicePoseArray> updatePoses()
{
	ros::Time currentTimeRos = ros::Time::now();
	std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();		
	shared_ptr<opt_msgs::ARDevicePoseArray> msg = make_shared<opt_msgs::ARDevicePoseArray>();
	msg->header.stamp  = currentTimeRos;
	msg->header.frame_id = "";
	visualization_msgs::MarkerArray markersMsg;
    std::lock_guard<std::mutex> lock(*handlersMutex);
	for (auto it = handlers->cbegin(); it != handlers->cend();)
	{
		if (std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - it->second->lastMessageTime).count() > no_update_timeout_millis)
		{
			//ROS_INFO_STREAM("removing "<<it->first);
			markersMsg.markers.push_back(buildDeletingMarker(it->first));
			handlers->erase(it++);    // or "it = m.erase(it)" since C++11
		}
		else
		{
			if(it->second->lastMessage)
			{
				opt_msgs::ARDevicePose devicePose;
				//ROS_INFO_STREAM("will send "<<it->first);
				devicePose.header = it->second->lastMessage->header;
				devicePose.pose = it->second->lastMessage->pose;
				devicePose.deviceId = it->first;
				msg->poses.push_back(devicePose);

				markersMsg.markers.push_back(buildArrowMarker(devicePose.pose.position.x, devicePose.pose.position.y,devicePose.pose.position.z,
						devicePose.deviceId,
						1,0,0,1, 
						0.4, 
						devicePose.header.frame_id,
						devicePose.pose.orientation.x,devicePose.pose.orientation.y,devicePose.pose.orientation.z,devicePose.pose.orientation.w));

			}
			else
			{
				//ROS_INFO_STREAM("no message yet from "<<it->first);				
			}
			++it;	
		}
	}

	pose_marker_pub.publish(markersMsg);
	return msg;
}


void deviceHeartbeatsCallback(const std_msgs::StringConstPtr& msg)
{
	string deviceName = msg->data;

    std::lock_guard<std::mutex> lock(*handlersMutex);

	auto it = handlers->find(deviceName);
	if(it==handlers->end())//if it doesn't exist, create it
	{
		ROS_INFO_STREAM("New device detected, id="<<deviceName);
		shared_ptr<PoseReceiver> newHandler = std::make_shared<PoseReceiver>(deviceName,nodeHandle,handlers,handlersMutex);
		
		handlers->insert(std::map<string, shared_ptr<PoseReceiver>>::value_type(newHandler->deviceId,newHandler));
		ROS_INFO_STREAM("Started subsccriber for device "<<newHandler->deviceId);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	nodeHandle = make_shared<ros::NodeHandle>();
	ROS_INFO_STREAM("starting "<<NODE_NAME);

	handlers = make_shared<map<string, shared_ptr<PoseReceiver>>>();
	handlersMutex = make_shared<mutex>();

	pose_marker_pub = nodeHandle->advertise<visualization_msgs::MarkerArray>(ardevice_poses_output_markers_topic, 1);
	ROS_INFO_STREAM("Advertised "<<ros::names::remap(ardevice_poses_output_markers_topic));

	ros::Publisher publisher = nodeHandle->advertise<opt_msgs::ARDevicePoseArray>(ardevice_poses_output_topic, 1000);
	ROS_INFO_STREAM("Advertised "<<ros::names::remap(ardevice_poses_output_topic));

	ros::Subscriber sub = nodeHandle->subscribe(devices_heartbeats_topicName, 10, deviceHeartbeatsCallback);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(devices_heartbeats_topicName));

	ros::Rate loop_rate(30);


	while (ros::ok())
	{

		shared_ptr<opt_msgs::ARDevicePoseArray> msgPtr = updatePoses();


		publisher.publish(*msgPtr);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


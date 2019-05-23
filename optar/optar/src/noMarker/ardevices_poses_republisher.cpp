/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * ROS node that aggregates the AR devices poses in a common topic.
 * Reads the poses as they are published by the single AR devices on their
 * respective topics and publishes them regularly as a list, also publishes
 * rviz markers for them.
 *
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

/** ROS node name */
const string NODE_NAME 								= "ardevices_poses_republisher";
/** name of the output topic, it should be remapped */
const string ardevice_poses_output_topic			= "ardevices_poses_output";
/** name of the output marker topic for rviz, it should be remapped */
const string ardevice_poses_output_markers_topic	= "ardevices_poses_output_markers";
/** name of the input heartbeats topic, it should be remapped */
const string devices_heartbeats_topicName			= "heartbeats_topic" ;

/** If no message is received from a device for this length of time the device is assumed dead and
    its pose doesn't get published anymore */
static double no_update_timeout_millis = 2000;


/**
 * Class that handles a single device, remembering its pose and the last time we heard from it
 */
class PoseReceiver
{
private:
	/** Subscriber for pose messages from the device */
	ros::Subscriber subscriber;
	/** mutex. TODO: should be a private mutex just for this object, should have getters for the private methods that use it */
	shared_ptr<mutex> handlersMutex;
public:

	/** ID of the AR device this object is handling */
	string deviceId;
	/** Last time we received a message from the AR device */
	std::chrono::steady_clock::time_point lastMessageTime;
	/** The last message we received */
	geometry_msgs::PoseStamped lastMessage;
	/** If we ever received a message */
	bool didReceivePose;

	/**
	 * Callback for receiving new pose messages from the AR device
	 * @param poseMsg The message from the device
	 */
	void poseCallback(const geometry_msgs::PoseStampedConstPtr poseMsg)
	{
		//ROS_INFO_STREAM(""<<deviceId<<": received ");
		std::lock_guard<std::mutex> lock(*handlersMutex);
		lastMessageTime = std::chrono::steady_clock::now();
		lastMessage = *poseMsg;

		tf::Pose convertedPose_tf = convertCameraPoseArcoreToRos(poseMsg->pose);
	  tf::poseTFToMsg(convertedPose_tf,lastMessage.pose);
		didReceivePose = true;
	}

	/**
	 * Builds the pose receiver
	 * @param deviceId      Device ID for the AR device the object will handle
	 * @param nodeHandle    ROS node handle
	 * @param handlersMutex Mutex of the "handlers" variable. TODO: this shouldn't be needed
	 */
	PoseReceiver(string deviceId,
				std::shared_ptr<ros::NodeHandle> nodeHandle,
				shared_ptr<mutex> handlersMutex)
	{
		this->deviceId = deviceId;
		this->handlersMutex = handlersMutex;

		string poseTopic = "/optar/"+deviceId+"/pose";
		subscriber = nodeHandle->subscribe(poseTopic, 10, &PoseReceiver::poseCallback, this);
		lastMessageTime = std::chrono::steady_clock::now();
	}
};

/** handlers for the active AR device we are monitoring */
shared_ptr<map<string, shared_ptr<PoseReceiver>>> handlers;
/** mutex for the ::handlers variable */
shared_ptr<mutex> handlersMutex;
/** ROS node handle */
shared_ptr<ros::NodeHandle> nodeHandle;
/** publisher for the rviz markers, TODO: this should be a global variable, should
    be a local varoabel in ::main() */
ros::Publisher	pose_marker_pub;


/**
 * Builds a list of the current device poses.
 * At the same time removes the handlers of inactive device (devices that didn't
 * publish anything in a while)
 * TODO: this also publishes the rviz markers of the poses. This should be done in the ::main()
 * @return the list of the AR device poses, as a ROS message, ready to be published
 */
shared_ptr<opt_msgs::ARDevicePoseArray> update()
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
			if(it->second->didReceivePose)
			{
				opt_msgs::ARDevicePose devicePose;
				//ROS_INFO_STREAM("will send "<<it->first);
				devicePose.header = it->second->lastMessage.header;
				devicePose.pose = it->second->lastMessage.pose;
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

/**
 * Callback for the heartbeats messages sent by the AR devices.
 * Adds newly discovered devices and starts listening to their poses.
 * @param msg Heartbeat message from an AR device, it contains the device ID
 */
void deviceHeartbeatsCallback(const std_msgs::StringConstPtr& msg)
{
	string deviceName = msg->data;

	std::lock_guard<std::mutex> lock(*handlersMutex);

	auto it = handlers->find(deviceName);
	if(it==handlers->end())//if it doesn't exist, create it
	{
		ROS_INFO_STREAM("New device detected, id="<<deviceName);
		shared_ptr<PoseReceiver> newHandler = std::make_shared<PoseReceiver>(deviceName,nodeHandle,handlersMutex);

		handlers->insert(std::map<string, shared_ptr<PoseReceiver>>::value_type(newHandler->deviceId,newHandler));
		ROS_INFO_STREAM("Started subscriber for device "<<newHandler->deviceId);
	}
}

/**
 * Main funciton for the ROS node.
 *  - Sets up the node
 *  - Advertises the output topics
 *  - Subscribes to the devices heartbeats
 *  - Loops to publish the device poses regularly
 * @param  argc
 * @param  argv
 * @return
 */
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

		shared_ptr<opt_msgs::ARDevicePoseArray> msgPtr = update();


		publisher.publish(*msgPtr);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

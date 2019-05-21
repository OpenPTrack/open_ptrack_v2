/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * ROS node that receives raw single-camera phone pose estimates and computes the
 * ARCore registration form them
 *
 */


#include <ros/ros.h>
#include <opt_msgs/ARDevicePoseEstimate.h>
#include "../ARDevicesManager.hpp"
#include "ARDeviceRegistrationEstimator.hpp"

using namespace std;

/** ROS node name */
const string NODE_NAME 								= "ardevices_registration_estimator";
/** Input topic on which we receive all the raw estimates, it should be remapped */
static const string inputRawPnPPoseTopic		= "input_raw_pose_estimate_topic";

/** ROS node handle */
shared_ptr<ros::NodeHandle> nodeHandle;
/** list of the registration estimators */
map<string,shared_ptr<ARDeviceRegistrationEstimator>> estimators;

void onPoseReceived(const std::string& deviceName, const geometry_msgs::PoseStampedConstPtr& poseMsg)
{
	auto it = estimators.find(deviceName);
	if(it==estimators.end())//if it doesn't exist, create it
	{
		ROS_ERROR_STREAM("Received ARCore pose for disconnected device. Ignoring it.");
		return;
	}
	it->second->onArcorePoseReceived(*poseMsg);
}

void onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimatePtr& inputPoseMsg)
{
	ROS_INFO("Received raw PnP pose estimate");
	auto it = estimators.find(inputPoseMsg->deviceId);
  if(it==estimators.end())//if it doesn't exist, create it
  {
		ROS_ERROR_STREAM("Received pnp pose for disconnected device. Ignoring it.");
		return;
	}

	it->second->onPnPPoseReceived(*inputPoseMsg);
}

void onArDeviceConnected(const string& deviceName)
{
	ROS_INFO_STREAM("New device connected with id = "<<deviceName);
	shared_ptr<ARDeviceRegistrationEstimator> newEstimator = make_shared<ARDeviceRegistrationEstimator>(deviceName, ros::Duration(1.5));
	estimators.insert(std::map<string, shared_ptr<ARDeviceRegistrationEstimator>>::value_type(deviceName,newEstimator));
	newEstimator->start(nodeHandle);
}

void onArDeviceDisconnected(const string& deviceName)
{
	ROS_INFO_STREAM("Device disconnected, id = "<<deviceName);
	estimators.erase(deviceName);
}
/**
 * Main function for the ROS node.
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	nodeHandle = make_shared<ros::NodeHandle>();
	ROS_INFO_STREAM("starting "<<NODE_NAME);

	ros::Subscriber pnpPosesReceiver = nodeHandle->subscribe(inputRawPnPPoseTopic, 100, onPnPPoseReceived);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(inputRawPnPPoseTopic));

	ARDevicesManager devicesManager(true, ros::Duration(5));
	devicesManager.setOnPoseReceivedCallback(onPoseReceived);
	devicesManager.setOnDeviceConnected(onArDeviceConnected);
	devicesManager.setOnDeviceDisconnected(onArDeviceDisconnected);
	devicesManager.start(nodeHandle);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::spin();

	return 0;
}

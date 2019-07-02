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
#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <optar/OptarRegistrationEstimatorParametersConfig.h>

using namespace std;

/** ROS node name */
const string NODE_NAME 								= "ardevices_registration_estimator";
/** Input topic on which we receive all the raw estimates, it should be remapped */
static const string inputRawPnPPoseTopic		= "input_raw_pose_estimate_topic";

/** ROS node handle */
shared_ptr<ros::NodeHandle> nodeHandle;
/** list of the registration estimators */
map<string,shared_ptr<ARDeviceRegistrationEstimator>> estimators;
/** mutex for the estimators map */
std::timed_mutex estimatorsMutex;

/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double positionProcessVariance = 0.000001;
/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double positionMeasurementVariance = 10;
/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double orientationProcessVariance = 0.001;
/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double orientationMeasurementVariance = 10;
/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double pnpMeasurementVarianceFactor = 1;
/** parameter for the poses filtering, see cfg/OptarRegistrationEstimatorParameters.cfg */
double arcoreMeasurementVarianceFactor = 20;


/**
 * Callback for the dynamic parameters onfiguration
 * @param config New parameters to be used
 * @param level  [description]
 */
void dynamicParametersCallback(optar::OptarRegistrationEstimatorParametersConfig &config, uint32_t level)
{
	positionProcessVariance = config.positionProcessVariance;
	positionMeasurementVariance = config.positionMeasurementVariance;
	orientationProcessVariance = config.orientationProcessVariance;
	orientationMeasurementVariance = config.orientationMeasurementVariance;
	pnpMeasurementVarianceFactor = config.pnpMeasurementVarianceFactor;
	arcoreMeasurementVarianceFactor = config.arcoreMeasurementVarianceFactor;
}

/**
 * Called by the devicesManager when a new pose is received from an AR device
 * @param deviceName Id of the device
 * @param poseMsg    The pose
 */
void onPoseReceived(const std::string& deviceName, const geometry_msgs::PoseStampedConstPtr& poseMsg)
{

	std::unique_lock<std::timed_mutex> lock(estimatorsMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting.");
		return;
	}
	auto it = estimators.find(deviceName);
	if(it==estimators.end())//if it doesn't exist, create it
	{
		ROS_ERROR_STREAM("Received ARCore pose for disconnected device. Ignoring it.");
		return;
	}

	//ROS_INFO_STREAM("Received ARCore pose  t="<<poseMsg->header.stamp);
	it->second->onArcorePoseReceived(*poseMsg);
}

/**
 * Callback for receiving PnP pose esitmates for the AR devices
 * @param inputPoseMsg The new Pose estimate
 */
void onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimatePtr& inputPoseMsg)
{
	ROS_INFO_STREAM("Received raw PnP pose estimate, t="<<inputPoseMsg->cameraPose.header.stamp);

	std::unique_lock<std::timed_mutex> lock(estimatorsMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting.");
		return;
	}
	auto it = estimators.find(inputPoseMsg->deviceId);
  if(it==estimators.end())//if it doesn't exist
  {
		ROS_ERROR_STREAM("Received pnp pose for disconnected device. Ignoring it.");
		return;
	}

	it->second->onPnPPoseReceived(*inputPoseMsg);
}

/**
 * Called by the devicesManager when a new AR device is detected
 * @param deviceName ID of the device
 */
void onArDeviceConnected(const string& deviceName)
{

	std::unique_lock<std::timed_mutex> lock(estimatorsMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting.");
		return;
	}

	ROS_INFO_STREAM("New device connected with id = "<<deviceName);
	shared_ptr<ARDeviceRegistrationEstimator> newEstimator = make_shared<ARDeviceRegistrationEstimator>(deviceName, ros::Duration(15));
	newEstimator->setupParameters( positionProcessVariance,
																 positionMeasurementVariance,
																 orientationProcessVariance,
																 orientationMeasurementVariance,
																 pnpMeasurementVarianceFactor,
																 arcoreMeasurementVarianceFactor);

	estimators.insert(std::map<string, shared_ptr<ARDeviceRegistrationEstimator>>::value_type(deviceName,newEstimator));
	newEstimator->start(nodeHandle);
	ROS_INFO_STREAM("Built estimator for device "<<deviceName);
}

/**
 * Called by the devicesManager when an AR device disconnects
 * @param deviceName ID of the device
 */
void onArDeviceDisconnected(const string& deviceName)
{

	std::unique_lock<std::timed_mutex> lock(estimatorsMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting.");
		return;
	}

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


	dynamic_reconfigure::Server<optar::OptarRegistrationEstimatorParametersConfig> server;
	dynamic_reconfigure::Server<optar::OptarRegistrationEstimatorParametersConfig>::CallbackType bindedDynamicParametersCallback;
	bindedDynamicParametersCallback = boost::bind(&dynamicParametersCallback, _1, _2);
	server.setCallback(bindedDynamicParametersCallback);

	ros::Subscriber pnpPosesReceiver = nodeHandle->subscribe(inputRawPnPPoseTopic, 100, onPnPPoseReceived);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(inputRawPnPPoseTopic));

	ARDevicesManager devicesManager(true, ros::Duration(15));
	devicesManager.setOnPoseReceivedCallback(onPoseReceived);
	devicesManager.setOnDeviceConnected(onArDeviceConnected);
	devicesManager.setOnDeviceDisconnected(onArDeviceDisconnected);
	devicesManager.start(nodeHandle);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::waitForShutdown();

	return 0;
}

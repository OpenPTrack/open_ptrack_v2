/**
 * @file
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 * Implementation of the ardevices_registration_aggregator ROS node
 * This node is responsible for aggregating the raw estimates produced by the
 * ardevices_pose_estimator_single_camera_raw nodes. So it receives all the raw
 * estimates, aggregates them and filters them to generate a unique registration
 * for each AR device.
 *
 * All the raw estimates are received on the same topic
 */

#include <ros/ros.h>
#include <chrono>
#include <mutex>
#include <memory> //shared_ptr
#include <opt_msgs/ARDevicePoseEstimate.h>
#include <std_msgs/String.h>

#include "TransformFilterKalman.hpp"
#include "../../utils.hpp"

#include <dynamic_reconfigure/server.h>
#include <optar/OptarAggregatorParametersConfig.h>


using namespace std;

/** ROS node name */
static const string NODE_NAME 	= "ardevices_registration_aggregator";
/** Input topic on which we receive all the raw estimates, it should be remapped */
static const string inputRawPoseTopic		= "input_raw_pose_estimate_topic";
/** Input topic on which we receive all the AR devices heartbeats, it should be remapped */
static const string inputHearbeatTopic = "input_heartbeats_topic";

/** Number of additional threads to use. We will have these and also the main thread,
    which keeps looping */
static const int threadsNumber = 2;

/** If a device doesn't publish a heatbeat for this length of time it is removed
    and forgotten */
static unsigned long handler_no_msg_timeout_millis = 5000;
/** Number of frames used to start up the filtering, to really useful, should
    stay low */
static int startupFramesNum = 3;
/** Filters out new estimates that are too far from the current estimate.
    Should stay quite big to avoid getting stuck in a bad estimate */
static double estimate_distance_thresh_meters = 5;

/**
 * Couples an estimate filter with a timestamp to keep track of the last time we
 *  heard from related the AR device.
 */
class FilterTimestampCouple
{
public:
	/** The last time we heard from the AR device */
	std::chrono::steady_clock::time_point lastTimeUsed;
	/** The filter for the AR device */
	shared_ptr<TransformFilterKalman> filter;

	/**
	 * @param filter The filter to use
	 */
	FilterTimestampCouple(shared_ptr<TransformFilterKalman> filter)
	{
		this->filter = filter;
	}

	/**
	 * Returns the length of time since FilterTimestampCouple#lastTimeUsed
	 * @return The time in milliseconds
	 */
	unsigned long getTimeSinceUsedMillis()
	{
		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		return  std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimeUsed).count();
	}

};

/** Filters for all the active devices registration estimates */
static std::map<string, shared_ptr<FilterTimestampCouple>> filters;
/** Mutex for the ::filters variable */
static std::timed_mutex filtersMutex;


/**
 * Callback for updating the parameters via dynamic_reconfigure
 * @param config The new parameters
 * @param level  Not used
 */
void dynamicParametersCallback(optar::OptarAggregatorParametersConfig &config, uint32_t level)
{
	ROS_INFO_STREAM("Reconfigure Request: "<<endl<<
			"estimate distance threshold = "<<config.estimate_distance_thresh_meters<<endl<<
			"handler_no_msg_timeout_secs = "<<config.handler_no_msg_timeout_secs<<endl<<
			"startup frames number = "<<config.startup_frames_num);


	startupFramesNum				= config.startup_frames_num;
	estimate_distance_thresh_meters	= config.estimate_distance_thresh_meters;
	handler_no_msg_timeout_millis	= config.handler_no_msg_timeout_secs*1000;

	for(auto const& keyValuePair: filters)
	{
		keyValuePair.second->filter->setupParameters(startupFramesNum,	estimate_distance_thresh_meters);
	}
}

/**
 * Callback for receiving the raw estimations from the ardevices_pose_estimator_single_camera_raw
 * nodes
 * @param inputRegistration Raw estimation from a single camera node
 */
void onRawPoseReceived(const opt_msgs::ARDevicePoseEstimatePtr& inputPoseMsg)
{
	string deviceId = inputPoseMsg->deviceId;

	std::unique_lock<std::timed_mutex> lock(filtersMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("onRawPoseReceived(): failed to get mutex. Skipping msg for "<<deviceId);
		return;
	}

	ROS_INFO_STREAM("received input pose from "<<inputPoseMsg->fixed_sensor_name<<" for device "<<inputPoseMsg->deviceId);




	tf::Pose phonePoseTf_world;
	tf::poseMsgToTF(inputPoseMsg->cameraPose.pose,phonePoseTf_world);
	//You don't belive me? Dim:
	// let phonePoseArcoreFrameConverted = Pa
	// let arcoreWorld = A
	// let phonePoseTf_world = Pr
	//
	// A*Pa*0 = Pr*0
	// so:
	// A*Pa = Pr
	// so:
	// A*Pa*Pa^-1 = Pr*Pa^-1
	// so:
	// A = Pr*Pa^-1

	tf::Pose arcoreWorld = phonePoseTf_world * convertCameraPoseArcoreToRos(inputPoseMsg->cameraPose_mobileFrame).inverse();

	if(!isPoseValid(arcoreWorld))
	{
		ROS_WARN_STREAM("Dropping pose estimate as it is invalid");
		return;
	}

	publishTransformAsTfFrame(phonePoseTf_world,inputPoseMsg->deviceId+"_estimate_"+inputPoseMsg->fixed_sensor_name,"/world",inputPoseMsg->header.stamp);
	publishTransformAsTfFrame(arcoreWorld,inputPoseMsg->deviceId+"_world_"+inputPoseMsg->fixed_sensor_name,"/world",inputPoseMsg->header.stamp);
	tf::StampedTransform inputTransform(arcoreWorld, inputPoseMsg->header.stamp, "/world", inputPoseMsg->deviceId+"_world");



	auto it = filters.find(deviceId);
	if(it==filters.end())//if it dowsn't exist, create it
	{
		ROS_INFO_STREAM("New device detected, id="<<deviceId);
		shared_ptr<TransformFilterKalman> newFilter = std::make_shared<TransformFilterKalman>(1e-5,1,1, 5, 5);
		newFilter->setupParameters(startupFramesNum,estimate_distance_thresh_meters);
		shared_ptr<FilterTimestampCouple> couple = make_shared<FilterTimestampCouple>(newFilter);

		filters.insert(std::map<string, shared_ptr<FilterTimestampCouple>>::value_type(deviceId,couple));
		ROS_INFO_STREAM("Built new filter for device  "<<deviceId);
	}
	//now it should always work
	shared_ptr<FilterTimestampCouple> couple = filters.find(deviceId)->second;
	couple->lastTimeUsed = std::chrono::steady_clock::now();

	tf::Pose filteredRegistration = couple->filter->update(inputTransform);
	publishTransformAsTfFrame(filteredRegistration,
		inputTransform.child_frame_id_+"_filtered",
		inputTransform.frame_id_,
		inputTransform.stamp_);
	ROS_INFO_STREAM("Published filterd transform for device "<<inputPoseMsg->deviceId);
}

/**
 * Removes the filters for inactive devices. I.e. devices that didn't publish an heartbeat
 * in ::handler_no_msg_timeout_millis milliseconds
 */
void removeOldFilters()
{
	std::unique_lock<std::timed_mutex> lock(filtersMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("removeOldFilters(): failed to get mutex. Skipping.");
		return;
	}

	for (auto it = filters.cbegin(); it != filters.cend();)
	{
		unsigned int millisSinceMsg = it->second->getTimeSinceUsedMillis() ;

		if(millisSinceMsg>1500)
			ROS_INFO_STREAM(""<<it->first<< " no msg since "<<millisSinceMsg<< " ms");

		if (millisSinceMsg > handler_no_msg_timeout_millis)
		{
			ROS_INFO_STREAM("removing filter for "<<it->first);
			filters.erase(it++);
		}
		else
		{
			//ROS_INFO_STREAM("keeping handler for "<<it->first<< " no msg since "<<millisSinceMsg<< " ms");
			++it;
		}
	}
}

/**
 * Callback for the heartbeats from all the AR devices. Discovers the active devices
 * and sets up the filtering for them. Also, it keeps track of the active devices (those
 * that are still publishing)
 * @param msg Heartbeat from the AR device
 */
void onHeartbeatReceived(const std_msgs::StringConstPtr& msg)
{
	string deviceId = msg->data;
	std::unique_lock<std::timed_mutex> lock(filtersMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("onHeartbeatReceived(): failed to get mutex. Skipping.");
		return;
	}
	ROS_INFO_STREAM("Received heartbeat from "<<msg->data);

	auto it = filters.find(deviceId);
	if(it!=filters.end())
	{
		it->second->lastTimeUsed = std::chrono::steady_clock::now();
	}
}

/**
 * Main method for the ROS node.
 * Sets up listeners and avertises the output topics.
 * Loops to remove inactive devices.
 *
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;

	ROS_INFO_STREAM("starting "<<NODE_NAME);

	dynamic_reconfigure::Server<optar::OptarAggregatorParametersConfig> server;
	dynamic_reconfigure::Server<optar::OptarAggregatorParametersConfig>::CallbackType bindedDynamicParametersCallback;
	bindedDynamicParametersCallback = boost::bind(&dynamicParametersCallback, _1, _2);
	server.setCallback(bindedDynamicParametersCallback);


	//This will not launch concurrent callbacks even if we have multiple spinners. That would need to be enabled explicitly somewhere
	ros::Subscriber rawPoseSubscriber = nh.subscribe(inputRawPoseTopic, 100, onRawPoseReceived);
	ros::Subscriber heartbeatSubscriber = nh.subscribe(inputHearbeatTopic, 100, onHeartbeatReceived);

	ros::AsyncSpinner spinner(threadsNumber);
	spinner.start();

	ros::Rate loop_rate(0.5);

	while (ros::ok())
	{

		removeOldFilters();

		ros::spinOnce();
		loop_rate.sleep();
	}

}

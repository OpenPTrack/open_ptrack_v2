/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Implementation of the ardevices_registration_aggregator ROS node
 * This node is responsible for aggregating the raw estimation produced by the ardevices_registration_single_camera_raw
 * nodes. So it receives all the raw estimations, aggregates them and filters them to generate a unique registration
 * for each AR device
 */

#include <ros/ros.h>
#include <chrono>
#include <mutex>
#include <memory> //shared_ptr
#include <opt_msgs/ARDeviceRegistration.h>
#include <std_msgs/String.h>

#include "TransformFilterKalman.hpp"
#include "../utils.hpp"

#include <dynamic_reconfigure/server.h>
#include <optar/OptarAggregatorParametersConfig.h>


using namespace std;

static const string NODE_NAME 	= "ardevices_registration_aggregator";
static const string inputRawEstimationTopic		= "input_raw_transform_topic";
static const string inputHearbeatTopic = "input_heartbeats_topic";

static const int threadsNumber = 2;

static unsigned long  handler_no_msg_timeout_millis = 5000;
static int startupFramesNum = 3;
static double estimate_distance_thresh_meters = 5;

class FilterTimestampCouple
{
public:
	std::chrono::steady_clock::time_point lastTimeUsed;
	shared_ptr<TransformFilterKalman> filter;

	FilterTimestampCouple(shared_ptr<TransformFilterKalman> filter)
	{
		this->filter = filter;
	}

	unsigned long getTimeSinceUsedMillis()
	{
		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
		return  std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimeUsed).count();
	}

};

static std::map<string, shared_ptr<FilterTimestampCouple>> filters;
static std::timed_mutex filtersMutex;



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


void onRegistrationReceived(const opt_msgs::ARDeviceRegistrationConstPtr& inputRegistration)
{
	string deviceId = inputRegistration->deviceId;

	std::unique_lock<std::timed_mutex> lock(filtersMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("onRegistrationReceived(): failed to get mutex. Skipping msg for "<<deviceId);
		return;
	}

	ROS_INFO_STREAM("received input transform from "<<inputRegistration->fixed_sensor_name<<" for device "<<inputRegistration->deviceId);


	tf::StampedTransform inputTransform;
	tf::transformStampedMsgToTF(inputRegistration->transform, inputTransform);
	if(!isPoseValid(inputTransform))
	{
		ROS_WARN_STREAM("Dropping transform as it is invalid");
		return;
	}

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
		inputRegistration->transform.child_frame_id+"_filtered",
		inputRegistration->transform.header.frame_id,
		inputRegistration->transform.header.stamp);
	ROS_INFO_STREAM("Published filterd transform for device "<<inputRegistration->deviceId);
}


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

void onHeartbeatReceived(const std_msgs::StringConstPtr& msg)
{
	string deviceId = msg->data;
	std::unique_lock<std::timed_mutex> lock(filtersMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("onHeartbeatReceived(): failed to get mutex. Skipping.");
		return;
	}

	auto it = filters.find(deviceId);
	if(it!=filters.end())
	{
		it->second->lastTimeUsed = std::chrono::steady_clock::now();
	}
}


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
	ros::Subscriber registrationSubscriber = nh.subscribe(inputRawEstimationTopic, 1, onRegistrationReceived);
	ros::Subscriber heartbeatSubscriber = nh.subscribe(inputHearbeatTopic, 1, onHeartbeatReceived);

	if(threadsNumber>1)
	{
		ros::AsyncSpinner spinner(threadsNumber-1);
		spinner.start();
	}

	ros::Rate loop_rate(1);

	while (ros::ok())
	{

		removeOldFilters();

		ros::spinOnce();
		loop_rate.sleep();
	}

}



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

#include "TransformFilterKalman.hpp"
#include "../utils.hpp"


using namespace std;

static const string NODE_NAME 	= "ardevices_registration_aggregator";
static const string inputRawEstimationTopic		= "input_raw_transform_topic";

static const int threadsNumber = 2;

static unsigned long  handler_no_msg_timeout_millis = 5000;
static int startupFramesNum = 3;
static double estimateDistanceThreshold_meters = 5;

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
		newFilter->setupParameters(startupFramesNum,estimateDistanceThreshold_meters);
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


int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;
	
	ROS_INFO_STREAM("starting "<<NODE_NAME);
	
	//This will not launch concurrent callbacks even if we have multiple spinners. That would need to be enabled explicitly somewhere
	ros::Subscriber sub = nh.subscribe(inputRawEstimationTopic, 1, onRegistrationReceived);

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



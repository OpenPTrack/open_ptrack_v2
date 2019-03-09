
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <chrono>
#include <tf/transform_listener.h>
#include <thread>         // std::this_thread::sleep_for
#include <mutex>

#include "../utils.hpp"
#include "ARDeviceHandler.hpp"



using namespace std;

void ARDeviceHandler::imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg)
{
	ROS_INFO_STREAM("imagesCallback for device "<<ARDeviceId);
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.imagesCallback(): failed to get mutex. Skipping message. ARDeviceId = "<<ARDeviceId);
		return;
	}

	int r = estimator->update(arcoreInputMsg,kinectInputCameraMsg,kinectInputDepthMsg,cameraInfo);
	if(r<0)
	{
		ROS_WARN_STREAM("estimation for device "<<ARDeviceId<<" failed with code "<<r);
	}
	else
	{
		publishTransformAsTfFrame(estimator->getEstimation(),estimator->getARDeviceId()+"_world_filtered","/world",arcoreInputMsg->header.stamp);
		ROS_INFO("Published transform");
	}
}

ARDeviceHandler::ARDeviceHandler(std::string ARDeviceId, std::string cameraRgbTopicName, std::string cameraDepthTopicName, std::string cameraInfoTopicName)
{

	this->ARDeviceId = ARDeviceId;

	this->cameraRgbTopicName = cameraRgbTopicName;
	this->cameraDepthTopicName = cameraDepthTopicName;
	this->cameraInfoTopicName = cameraInfoTopicName;

	arDeviceCameraMsgTopicName = "/optar/"+ARDeviceId+"/camera";

}



int ARDeviceHandler::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.start(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}

	


	//get camera info
	boost::shared_ptr<sensor_msgs::CameraInfo const> cameraInfoPtr;
	ROS_INFO_STREAM("waiting for cameraInfo on topic "<<ros::names::remap(cameraInfoTopicName));
	cameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopicName);
	if(cameraInfoPtr == NULL)
	{
		ROS_ERROR_STREAM("couldn't get camera_info from topic "<<ros::names::remap(cameraInfoTopicName)<<", did the node shut down? I'll shut down.");
		return -2;
	}
	cameraInfo = *cameraInfoPtr;


	//get camera to world transform
	ros::Time targetTime;
	tf::TransformListener listener;
	std::string inputFrame = cameraInfo.header.frame_id;
	std::string targetFrame = "/world";
	ros::Duration timeout = ros::Duration(10.0);
	bool retry=true;
	int count=0;
	ROS_INFO_STREAM("getting transform from "<<inputFrame<<" to "<<targetFrame);
	std::this_thread::sleep_for (std::chrono::seconds(2));//sleep two second to let tf start
	do
	{
		std::string failReason;
		targetTime = ros::Time(0);//the latest available
		bool r = listener.waitForTransform( targetFrame,inputFrame, targetTime, timeout, ros::Duration(0.01),&failReason);
		if(!r)
		{
			ROS_INFO_STREAM("can't transform because: "<<failReason);
			if(count>10)
				return -3;
			ROS_INFO("retrying");
		}
		else
			retry=false;
		count++;
	}while(retry);
	tf::StampedTransform transformKinectToWorldNotMsg = tf::StampedTransform();
	listener.lookupTransform(targetFrame, inputFrame, targetTime, transformKinectToWorldNotMsg);
	tf::transformStampedTFToMsg(transformKinectToWorldNotMsg,transformKinectToWorld);
	ROS_INFO("Got transform from %s to %s ",inputFrame.c_str(), targetFrame.c_str());








	estimator = std::make_shared<ARDeviceRegistrationEstimator>(ARDeviceId, *nodeHandle, transformKinectToWorld);
	estimator->setupParameters(pnpReprojectionError,
								pnpConfidence,
								pnpIterations,
								matchingThreshold,
								reprojectionErrorDiscardThreshold,
								orbMaxPoints,
								orbScaleFactor,
								orbLevelsNumber,
								startupFramesNum,
								phoneOrientationDifferenceThreshold_deg,
								estimateDistanceThreshold_meters,
								showImages,
								useCuda);




	arcoreCamera_sub = make_shared<message_filters::Subscriber<opt_msgs::ArcoreCameraImage>>(*nodeHandle, arDeviceCameraMsgTopicName, 1);
	kinect_img_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraRgbTopicName, 1);
	kinect_depth_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraDepthTopicName, 1);
	

	//instantiate and set up the policy
	MyApproximateSynchronizationPolicy policy = MyApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	policy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	policy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	policy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more

	//Instantiate a Synchronizer with our policy.
	synchronizer = std::make_shared<message_filters::Synchronizer<MyApproximateSynchronizationPolicy>>(MyApproximateSynchronizationPolicy(policy), *arcoreCamera_sub, *kinect_img_sub, *kinect_depth_sub);

	//registers the callback
	auto f = boost::bind( &ARDeviceHandler::imagesCallback, this, _1, _2, _3);

	ROS_INFO_STREAM("Setup listener for topics: "<<endl<<
		ros::names::remap(arDeviceCameraMsgTopicName)<<endl<<
		ros::names::remap(cameraRgbTopicName)<<endl<<
		ros::names::remap(cameraDepthTopicName)<<endl);
	synchronizer->registerCallback(f);
	return 0;
}

int ARDeviceHandler::stop()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.stop(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}
	ROS_INFO_STREAM("Stopping listener for "<<ARDeviceId);
	synchronizer.reset();
	return 0;
}

int ARDeviceHandler::setupParameters(double pnpReprojectionError,
					double pnpConfidence,
					double pnpIterations,
					double matchingThreshold,
					double reprojectionErrorDiscardThreshold,
					int orbMaxPoints,
					double orbScaleFactor,
					int orbLevelsNumber,
					unsigned int startupFramesNum,
					double phoneOrientationDifferenceThreshold_deg,
					double estimateDistanceThreshold_meters,
					bool showImages,
					bool useCuda)
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.setupParameters(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}
	this->pnpReprojectionError=pnpReprojectionError;
	this->pnpConfidence=pnpConfidence;
	this->pnpIterations=pnpIterations;
	this->matchingThreshold=matchingThreshold;
	this->reprojectionErrorDiscardThreshold=reprojectionErrorDiscardThreshold;
	this->orbMaxPoints=orbMaxPoints;
	this->orbScaleFactor=orbScaleFactor;
	this->orbLevelsNumber=orbLevelsNumber;
	this->startupFramesNum=startupFramesNum;
	this->phoneOrientationDifferenceThreshold_deg=phoneOrientationDifferenceThreshold_deg;
	this->estimateDistanceThreshold_meters=estimateDistanceThreshold_meters;
	this->showImages = showImages;
	this->useCuda = useCuda;

	if(estimator)
	{
		estimator->setupParameters(pnpReprojectionError,
					pnpConfidence,
					pnpIterations,
					matchingThreshold,
					reprojectionErrorDiscardThreshold,
					orbMaxPoints,
					orbScaleFactor,
					orbLevelsNumber,
					startupFramesNum,
					phoneOrientationDifferenceThreshold_deg,
					estimateDistanceThreshold_meters,
					showImages,
					useCuda);
	}
	return 0;
}


string ARDeviceHandler::getARDeviceId()
{
	return ARDeviceId;
}
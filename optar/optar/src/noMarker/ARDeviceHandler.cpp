
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
#include "opt_msgs/ARDeviceRegistration.h"



using namespace std;

void ARDeviceHandler::imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg)
{
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("imagesCallback for device "<<ARDeviceId);
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.imagesCallback(): failed to get mutex. Skipping message. ARDeviceId = "<<ARDeviceId);
		return;
	}


	lastTimeReceivedMessage = std::chrono::steady_clock::now();


	int r = estimator->imagesCallback(arcoreInputMsg,kinectInputCameraMsg,kinectInputDepthMsg,cameraInfo);
	if(r<0)
	{
		ROS_WARN_STREAM("estimation for device "<<ARDeviceId<<" failed with code "<<r);
	}
	else if(r>0)
	{
		ROS_INFO_STREAM("skipping frame, update returned "<<r);
	}
	else
	{
		//publishTransformAsTfFrame(estimator->getEstimation(),estimator->getARDeviceId()+"_world_filtered","/world",arcoreInputMsg->header.stamp);

		
		

		opt_msgs::ARDeviceRegistration outputRegistrationMsg;
		outputRegistrationMsg.deviceId = getARDeviceId();
		outputRegistrationMsg.fixed_sensor_name = fixed_sensor_name;
		outputRegistrationMsg.matches_number = estimator->getLastEstimateMatchesNumber();
		outputRegistrationMsg.reprojection_error = estimator->getLastEstimateReprojectionError();
		outputRegistrationMsg.transform = estimator->getLastEstimate();
		rawEstimationPublisher.publish(outputRegistrationMsg);

		tf::StampedTransform tfTransform;
		transformStampedMsgToTF(estimator->getLastEstimate(),tfTransform);
		ROS_INFO_STREAM("Published transform = "<<poseToString(tfTransform));
	}
}


void ARDeviceHandler::featuresCallback(const opt_msgs::ArcoreCameraFeaturesConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg)
{
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("featuresCallback for device "<<ARDeviceId);
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.imagesCallback(): failed to get mutex. Skipping message. ARDeviceId = "<<ARDeviceId);
		return;
	}

	lastTimeReceivedMessage = std::chrono::steady_clock::now();

	int r = estimator->featuresCallback(arcoreInputMsg,kinectInputCameraMsg,kinectInputDepthMsg,cameraInfo);
	if(r<0)
	{
		ROS_WARN_STREAM("estimation for device "<<ARDeviceId<<" failed with code "<<r);
	}
	else if(r>0)
	{
		ROS_INFO_STREAM("skipping frame, update returned "<<r);
	}
	else
	{
		//publishTransformAsTfFrame(estimator->getEstimation(),estimator->getARDeviceId()+"_world_filtered","/world",arcoreInputMsg->header.stamp);
		
		opt_msgs::ARDeviceRegistration outputRegistrationMsg;
		outputRegistrationMsg.deviceId = getARDeviceId();
		outputRegistrationMsg.fixed_sensor_name = fixed_sensor_name;
		outputRegistrationMsg.matches_number = estimator->getLastEstimateMatchesNumber();
		outputRegistrationMsg.reprojection_error = estimator->getLastEstimateReprojectionError();
		outputRegistrationMsg.transform = estimator->getLastEstimate();
		outputRegistrationMsg.header.stamp = arcoreInputMsg->header.stamp;
		rawEstimationPublisher.publish(outputRegistrationMsg);

		tf::StampedTransform tfTransform;
		transformStampedMsgToTF(estimator->getLastEstimate(),tfTransform);
		ROS_INFO_STREAM("Published transform = "<<poseToString(tfTransform));
	}
}

ARDeviceHandler::ARDeviceHandler(	std::string ARDeviceId, 
									std::string cameraRgbTopicName, 
									std::string cameraDepthTopicName, 
									std::string cameraInfoTopicName, 
									std::string debugImagesTopic, 
									std::string fixed_sensor_name, 
									std::string outputRawEstimationTopic,
									std::shared_ptr<FeaturesMemory> featuresMemory)
{

	this->ARDeviceId = ARDeviceId;
	this->fixed_sensor_name = fixed_sensor_name;
	this->cameraRgbTopicName = cameraRgbTopicName;
	this->cameraDepthTopicName = cameraDepthTopicName;
	this->cameraInfoTopicName = cameraInfoTopicName;
	this->debugImagesTopic = debugImagesTopic;
	this->outputRawEstimationTopic = outputRawEstimationTopic;
	this->featuresMemory = featuresMemory;

	arDeviceCameraMsgTopicName = "optar/"+ARDeviceId+"/camera";
	arDeviceFeaturesMsgTopicName = "optar/"+ARDeviceId+"/features";
	lastTimeReceivedMessage = std::chrono::steady_clock::now();

}

ARDeviceHandler::~ARDeviceHandler()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("~ARDeviceHandler(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
	}
	if(!stopped)
	{
		//ROS_INFO_STREAM("Stopping listener for "<<ARDeviceId);
		synchronizer.reset();
		featuresTpc_synchronizer.reset();
		//ROS_INFO_STREAM("Stopped listener for "<<ARDeviceId);
	}
}



int ARDeviceHandler::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.start(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}

	rawEstimationPublisher = nodeHandle->advertise<opt_msgs::ARDeviceRegistration>(outputRawEstimationTopic, 10);



	//get camera info
	boost::shared_ptr<sensor_msgs::CameraInfo const> cameraInfoPtr;
	ROS_INFO_STREAM("waiting for cameraInfo on topic "<<ros::names::remap(cameraInfoTopicName));
	cameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cameraInfoTopicName, ros::Duration(5));
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








	estimator = std::make_shared<ARDeviceRegistrationEstimator>(ARDeviceId, *nodeHandle, transformKinectToWorld, debugImagesTopic, fixed_sensor_name, featuresMemory);
	estimator->setupParameters(pnpReprojectionError,
								pnpConfidence,
								pnpIterations,
								matchingThreshold,
								reprojectionErrorDiscardThreshold,
								orbMaxPoints,
								orbScaleFactor,
								orbLevelsNumber,
								phoneOrientationDifferenceThreshold_deg,
								showImages,
								minimumMatchesNumber,
								enableFeaturesMemory);




	arcoreCamera_sub = make_shared<message_filters::Subscriber<opt_msgs::ArcoreCameraImage>>(*nodeHandle, arDeviceCameraMsgTopicName, 5);
	kinect_img_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraRgbTopicName, 30*10);
	kinect_depth_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraDepthTopicName, 30*10);
	

	//instantiate and set up the policy
	MyApproximateSynchronizationPolicy policy = MyApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	policy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	policy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	policy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more
	policy.setMaxIntervalDuration(ros::Duration(5,0));// 5 seconds

	//Instantiate a Synchronizer with our policy.
	synchronizer = std::make_shared<message_filters::Synchronizer<MyApproximateSynchronizationPolicy>>(MyApproximateSynchronizationPolicy(policy), *arcoreCamera_sub, *kinect_img_sub, *kinect_depth_sub);

	//registers the callback
	auto f = boost::bind( &ARDeviceHandler::imagesCallback, this, _1, _2, _3);

	ROS_INFO_STREAM("Setting up images synchronizer with topics:"<<endl<<
		ros::names::remap(arDeviceCameraMsgTopicName)<<endl<<
		ros::names::remap(cameraRgbTopicName)<<endl<<
		ros::names::remap(cameraDepthTopicName)<<endl);
	synchronizer->registerCallback(f);






	//instantiate and set up the policy
	FeaturesApproximateSynchronizationPolicy featuresPolicy = FeaturesApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	featuresPolicy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	featuresPolicy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	featuresPolicy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more
	featuresPolicy.setMaxIntervalDuration(ros::Duration(5,0));// 5 seconds


	featuresTpc_arcore_sub = make_shared<message_filters::Subscriber<opt_msgs::ArcoreCameraFeatures>>(*nodeHandle, arDeviceFeaturesMsgTopicName, 1);
	featuresTpc_kinect_img_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraRgbTopicName, 1);
	featuresTpc_kinect_depth_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, cameraDepthTopicName, 1);

	//Instantiate a Synchronizer with our policy.
	featuresTpc_synchronizer = std::make_shared<message_filters::Synchronizer<FeaturesApproximateSynchronizationPolicy>>(FeaturesApproximateSynchronizationPolicy(featuresPolicy), *featuresTpc_arcore_sub, *featuresTpc_kinect_img_sub, *featuresTpc_kinect_depth_sub);
	


	//registers the callback
	auto featuresCallbackFunction = boost::bind( &ARDeviceHandler::featuresCallback, this, _1, _2, _3);

	ROS_INFO_STREAM("Setting up images-features synchronizer with topics:"<<endl<<
		ros::names::remap(arDeviceFeaturesMsgTopicName)<<endl<<
		ros::names::remap(cameraRgbTopicName)<<endl<<
		ros::names::remap(cameraDepthTopicName)<<endl);
	featuresTpc_synchronizer->registerCallback(featuresCallbackFunction);

	lastTimeReceivedMessage = std::chrono::steady_clock::now();


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
	if(stopped)
		return 0;
	//ROS_INFO_STREAM("Stopping listener for "<<ARDeviceId);
	synchronizer.reset();
	featuresTpc_synchronizer.reset();
	//ROS_INFO_STREAM("Stopped listener for "<<ARDeviceId);
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
					double phoneOrientationDifferenceThreshold_deg,
					bool showImages,
					unsigned int minimumMatchesNumber,
					bool enableFeaturesMemory)
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
	this->phoneOrientationDifferenceThreshold_deg=phoneOrientationDifferenceThreshold_deg;
	this->showImages = showImages;
	this->minimumMatchesNumber = minimumMatchesNumber;
	this->enableFeaturesMemory = enableFeaturesMemory;

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
					phoneOrientationDifferenceThreshold_deg,
					showImages,
					minimumMatchesNumber,
					enableFeaturesMemory);
	}
	return 0;
}


string ARDeviceHandler::getARDeviceId()
{
	return ARDeviceId;
}


int ARDeviceHandler::millisecondsSinceLastMessage()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDeviceHandler.setupParameters(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	return  std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimeReceivedMessage).count();
}
/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * ARDevicePoseEstimatorSingleCamera methods implementation file
 */



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

#include "../../utils.hpp"
#include "ARDevicePoseEstimatorSingleCamera.hpp"
#include <opt_msgs/ARDevicePoseEstimate.h>



using namespace std;


/**
 * Computes a new estimation (if possible) using the precomputed features received from the AR
 * device. The estimation is computed using CameraPoseEstimator::featuresCallback()
 *
 * @param arcoreInputMsg       The message from the AR device
 * @param kinectInputCameraMsg Regular mono image message from the fixed camera
 * @param kinectInputDepthMsg  Depth image from the fixed camera
 */
void ARDevicePoseEstimatorSingleCamera::featuresCallback(const opt_msgs::ArcoreCameraFeaturesConstPtr& arcoreInputMsg,
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
		ROS_ERROR_STREAM("ARDevicePoseEstimatorSingleCamera.imagesCallback(): failed to get mutex. Skipping message. ARDeviceId = "<<ARDeviceId);
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




		opt_msgs::ARDevicePoseEstimate outputPoseMsg;
		outputPoseMsg.deviceId = getARDeviceId();
		outputPoseMsg.fixed_sensor_name = fixed_sensor_name;
		outputPoseMsg.matches_number = estimator->getLastEstimateMatchesNumber();
		outputPoseMsg.reprojection_error = estimator->getLastEstimateReprojectionError();
		outputPoseMsg.cameraPose = estimator->getLastPoseEstimate();
		outputPoseMsg.header.stamp = arcoreInputMsg->header.stamp;
		outputPoseMsg.cameraPose_mobileFrame = arcoreInputMsg->mobileFramePose;
		poseEstimatePublisher.publish(outputPoseMsg);

		ROS_INFO_STREAM("Published raw pose estimate "<<poseToString(outputPoseMsg.cameraPose.pose));




		tf::Pose phonePoseTf_world;
		tf::poseMsgToTF(estimator->getLastPoseEstimate().pose,phonePoseTf_world);
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

		tf::Pose arcoreWorld = phonePoseTf_world * convertCameraPoseArcoreToRos(arcoreInputMsg->mobileFramePose).inverse();

		if(!isPoseValid(arcoreWorld))
		{
			ROS_WARN_STREAM("Dropping transform estimate as it is invalid");
			return;
		}

		publishTransformAsTfFrame(phonePoseTf_world,ARDeviceId+"_estimate_"+fixed_sensor_name,"/world",arcoreInputMsg->header.stamp);
		publishTransformAsTfFrame(arcoreWorld,ARDeviceId+"_world_"+fixed_sensor_name,"/world",arcoreInputMsg->header.stamp);


		tf::StampedTransform stampedTransform(arcoreWorld, arcoreInputMsg->header.stamp, "/world", getARDeviceId()+"_world");
		geometry_msgs::TransformStamped stampedTransformMsg;
		tf::transformStampedTFToMsg(stampedTransform,stampedTransformMsg);


		tf::StampedTransform tfTransform;
		transformStampedMsgToTF(stampedTransformMsg,tfTransform);
		ROS_DEBUG_STREAM("Published raw transform = "<<poseToString(tfTransform));
	}
}


/**
 * Constructs a new ARDevicePoseEstimatorSingleCamera, you will neew to start it with ARDevicePoseEstimatorSingleCamera::start()
 *
 * @param ARDeviceId                    device Id of the handled device
 * @param fixedCameraMonoTopicName      Topic for receiving the mono images from the fixed camera
 * @param fixedCameraDepthTopicName     Topic for receiving the depth images from the fixed camera
 * @param cameraInfoTopicName           Topic for receiving the camera info for the fixed camera
 * @param fixed_sensor_name             Sensor name of the fixed camera
 * @param outputRawEstimationTopic      Topic on which to output the registration estimation
 * @param featuresMemory                FeatureMemory object to remember and share useful features
 */
ARDevicePoseEstimatorSingleCamera::ARDevicePoseEstimatorSingleCamera(	std::string ARDeviceId,
									std::string fixedCameraMonoTopicName,
									std::string fixedCameraDepthTopicName,
									std::string cameraInfoTopicName,
									std::string fixed_sensor_name,
									std::string poseEstimateTopicName,
									std::shared_ptr<FeaturesMemory> featuresMemory)
{

	this->ARDeviceId = ARDeviceId;
	this->fixed_sensor_name = fixed_sensor_name;
	this->fixedCameraMonoTopicName = fixedCameraMonoTopicName;
	this->fixedCameraDepthTopicName = fixedCameraDepthTopicName;
	this->cameraInfoTopicName = cameraInfoTopicName;
	this->poseEstimateTopicName = poseEstimateTopicName;
	this->featuresMemory = featuresMemory;

	arDeviceCameraMsgTopicName = "optar/"+ARDeviceId+"/camera";
	arDeviceFeaturesMsgTopicName = "optar/"+ARDeviceId+"/features";
	lastTimeReceivedMessage = std::chrono::steady_clock::now();

}

/**
 * Destructs the object unsubscribing from the topics if needed
 */
ARDevicePoseEstimatorSingleCamera::~ARDevicePoseEstimatorSingleCamera()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("~ARDevicePoseEstimatorSingleCamera(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
	}
	if(!stopped)
	{
		//ROS_INFO_STREAM("Stopping listener for "<<ARDeviceId);
		synchronizer.reset();
		featuresTpc_synchronizer.reset();
		//ROS_INFO_STREAM("Stopped listener for "<<ARDeviceId);
	}
}


/**
 * Starts up the handler:
 *   - Advertises the output registration topic
 *   - Gets the fixed camera info
 *   - Gets the fixed camera position
 *   - Sets up the registration estimator
 *   - Sets up the input topics listeners
 *
 * @param  nodeHandle The ROS node handle
 * @return            0 on success, a negative value on fail
 */
int ARDevicePoseEstimatorSingleCamera::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDevicePoseEstimatorSingleCamera.start(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}

	poseEstimatePublisher = nodeHandle->advertise<opt_msgs::ARDevicePoseEstimate>(poseEstimateTopicName, 10);



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







	//setup the registration estimator
	estimator = std::make_shared<CameraPoseEstimator>(ARDeviceId, *nodeHandle, transformKinectToWorld, fixed_sensor_name, featuresMemory);
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


	//set up the input topics listeners



	//instantiate and set up the policy
	FeaturesApproximateSynchronizationPolicy featuresPolicy = FeaturesApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	featuresPolicy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	featuresPolicy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	featuresPolicy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more
	featuresPolicy.setMaxIntervalDuration(ros::Duration(5,0));// 5 seconds


	featuresTpc_arcore_sub = make_shared<message_filters::Subscriber<opt_msgs::ArcoreCameraFeatures>>(*nodeHandle, arDeviceFeaturesMsgTopicName, 1);
	featuresTpc_kinect_img_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, fixedCameraMonoTopicName, 1);
	featuresTpc_kinect_depth_sub = make_shared<message_filters::Subscriber<sensor_msgs::Image>>(*nodeHandle, fixedCameraDepthTopicName, 1);

	//Instantiate a Synchronizer with our policy.
	featuresTpc_synchronizer = std::make_shared<message_filters::Synchronizer<FeaturesApproximateSynchronizationPolicy>>(FeaturesApproximateSynchronizationPolicy(featuresPolicy), *featuresTpc_arcore_sub, *featuresTpc_kinect_img_sub, *featuresTpc_kinect_depth_sub);



	//registers the callback
	auto featuresCallbackFunction = boost::bind( &ARDevicePoseEstimatorSingleCamera::featuresCallback, this, _1, _2, _3);

	ROS_INFO_STREAM("Setting up images-features synchronizer with topics:"<<endl<<
		ros::names::remap(arDeviceFeaturesMsgTopicName)<<endl<<
		ros::names::remap(fixedCameraMonoTopicName)<<endl<<
		ros::names::remap(fixedCameraDepthTopicName)<<endl);
	featuresTpc_synchronizer->registerCallback(featuresCallbackFunction);

	lastTimeReceivedMessage = std::chrono::steady_clock::now();


	return 0;
}

/**
 * Unsubscribes from the input topics
 * @return 0 on success, a negative value on fail
 */
int ARDevicePoseEstimatorSingleCamera::stop()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDevicePoseEstimatorSingleCamera.stop(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
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

/**
 * Updates the parameters to be used
 * @param  pnpReprojectionError                    See CameraPoseEstimator#setupParameters()
 * @param  pnpConfidence                           See CameraPoseEstimator#setupParameters()
 * @param  pnpIterations                           See CameraPoseEstimator#setupParameters()
 * @param  matchingThreshold                       See CameraPoseEstimator#setupParameters()
 * @param  reprojectionErrorDiscardThreshold       See CameraPoseEstimator#setupParameters()
 * @param  orbMaxPoints                            See CameraPoseEstimator#setupParameters()
 * @param  orbScaleFactor                          See CameraPoseEstimator#setupParameters()
 * @param  orbLevelsNumber                         See CameraPoseEstimator#setupParameters()
 * @param  phoneOrientationDifferenceThreshold_deg See CameraPoseEstimator#setupParameters()
 * @param  showImages                              See CameraPoseEstimator#setupParameters()
 * @param  minimumMatchesNumber                    See CameraPoseEstimator#setupParameters()
 * @param  enableFeaturesMemory                    See CameraPoseEstimator#setupParameters()
 * @return                                         0 on success, a negative value on fail
 */
int ARDevicePoseEstimatorSingleCamera::setupParameters(double pnpReprojectionError,
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
		ROS_ERROR_STREAM("ARDevicePoseEstimatorSingleCamera.setupParameters(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
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

/**
 * Gets the device ID for the AR device handled by this object
 * @return The device ID
 */
string ARDevicePoseEstimatorSingleCamera::getARDeviceId()
{
	return ARDeviceId;
}

/**
 * Return the time since this handler last reeived a message from the AR device, in milliseconds
 * @return Th time in milliseconds
 */
int ARDevicePoseEstimatorSingleCamera::millisecondsSinceLastMessage()
{
	std::unique_lock<std::timed_mutex> lock(objectMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDevicePoseEstimatorSingleCamera.setupParameters(): failed to get mutex. ARDeviceId = "<<ARDeviceId);
		return -1;
	}
	std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
	return  std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimeReceivedMessage).count();
}

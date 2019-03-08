/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * This file implements a ros node which estimates the transformation between an ARCore 
 * coordinae frame and the ros tf /world frame
 */


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xphoto.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>


#include <dynamic_reconfigure/server.h>
#include <optar/OptarDynamicParametersConfig.h>

#include "../utils.hpp"
#include  "TransformKalmanFilter.hpp"
#include "ARDeviceRegistrationEstimator.hpp"


using namespace cv;
using namespace std;


const string NODE_NAME 						= "nomarker_position_estimator";
const string input_arcore_topic				= "arcore_camera";
const string input_kinect_camera_topic		= "kinect_camera" ;
const string input_kinect_depth_topic		= "kinect_depth";
const string input_kinect_camera_info_topic	= "kinect_camera_info" ;

std::map<string, shared_ptr<ARDeviceRegistrationEstimator>> estimators;
std::shared_ptr<ros::NodeHandle> nodeHandle;
geometry_msgs::TransformStamped transformKinectToWorld;




static double pnpReprojectionError = 5;
static double pnpConfidence = 0.99;
static double pnpIterations = 1000;
static double matchingThreshold = 25;
static double reprojectionErrorDiscardThreshold = 5;
static int orbMaxPoints = 500;
static double orbScaleFactor = 1.2;
static int orbLevelsNumber = 8;
static unsigned int startupFramesNum = 10;
static double phoneOrientationDifferenceThreshold_deg = 45;
static double estimateDistanceThreshold_meters = 5;
static bool showImages = false;
static bool useCuda = false;


void dynamicParametersCallback(optar::OptarDynamicParametersConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %f %f %f", 
            config.pnp_iterations,
            config.pnp_confidence, 
            config.pnp_reprojection_error);

	pnpReprojectionError = config.pnp_reprojection_error;
	pnpConfidence 					= config.pnp_confidence;
	pnpIterations 					= config.pnp_iterations;

	matchingThreshold 				= config.matching_threshold;
	reprojectionErrorDiscardThreshold = config.reprojection_discard_threshold;

	orbMaxPoints		= config.orb_max_points;
	orbScaleFactor	= config.orb_scale_factor;
	orbLevelsNumber	= config.orb_levels_number;

	startupFramesNum	= config.startup_frames_num;
	phoneOrientationDifferenceThreshold_deg	= config.phone_orientation_diff_thresh;
	showImages	= config.show_images;

	for(auto const& keyValuePair: estimators)
	{
		keyValuePair.second->setupParameters(pnpReprojectionError,
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
}

void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo)
{
	string deviceName = arcoreInputMsg->deviceId;
	auto it = estimators.find(deviceName);
	if(it==estimators.end())//if it dowsn't exist, create it
	{
		shared_ptr<ARDeviceRegistrationEstimator> newEstimator = make_shared<ARDeviceRegistrationEstimator>(deviceName, *nodeHandle, transformKinectToWorld);
		estimators.insert(std::map<string, shared_ptr<ARDeviceRegistrationEstimator>>::value_type(newEstimator->getARDeviceId(),newEstimator));
		estimators.at(deviceName)->setupParameters(pnpReprojectionError,
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
	shared_ptr<ARDeviceRegistrationEstimator> estimator = estimators.find(deviceName)->second;

	int r = estimator->update(arcoreInputMsg,kinectInputCameraMsg,kinectInputDepthMsg,kinectCameraInfo);
	if(r<0)
	{
		ROS_WARN_STREAM("estimation for device "<<deviceName<<" failed with code "<<r);
	}
	else
	{
		publishTransformAsTfFrame(estimator->getEstimation(),estimator->getARDeviceId()+"_world_filtered","/world",arcoreInputMsg->header.stamp);
		ROS_INFO("Published transform");
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	nodeHandle = make_shared<ros::NodeHandle>();
	ROS_INFO_STREAM("starting "<<NODE_NAME);


	dynamic_reconfigure::Server<optar::OptarDynamicParametersConfig> server;
	dynamic_reconfigure::Server<optar::OptarDynamicParametersConfig>::CallbackType bindedDynamicParametersCallback;
	bindedDynamicParametersCallback = boost::bind(&dynamicParametersCallback, _1, _2);
	server.setCallback(bindedDynamicParametersCallback);



	boost::shared_ptr<sensor_msgs::CameraInfo const> kinectCameraInfoPtr;
	sensor_msgs::CameraInfo kinectCameraInfo;
	ROS_INFO_STREAM("waiting for kinect cameraInfo on topic "<<ros::names::remap(input_kinect_camera_info_topic));
	kinectCameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_kinect_camera_info_topic);
	if(kinectCameraInfoPtr == NULL)
	{
		ROS_ERROR("couldn't get kinect's camera_info, did the node shut down? I'll shut down.");
		return 0;
	}
	kinectCameraInfo = *kinectCameraInfoPtr;


	ros::Time targetTime;
	tf::TransformListener listener;
	std::string inputFrame = kinectCameraInfo.header.frame_id;
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
				return -1;
			ROS_INFO("retrying");
		}
		else
			retry=false;
		count++;
	}while(retry);
	ROS_INFO("got transform");


	tf::StampedTransform transformKinectToWorldNotMsg = tf::StampedTransform();
	listener.lookupTransform(targetFrame, inputFrame, targetTime, transformKinectToWorldNotMsg);
	tf::transformStampedTFToMsg(transformKinectToWorldNotMsg,transformKinectToWorld);
	//ROS_INFO("got transform from %s to %s ",inputFrame.c_str(), targetFrame.c_str());


	message_filters::Subscriber<opt_msgs::ArcoreCameraImage> arcoreCamera_sub(*nodeHandle, input_arcore_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> kinect_img_sub(*nodeHandle, input_kinect_camera_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub(*nodeHandle, input_kinect_depth_topic, 1);
	
	// Synchronization policy for having a callback that receives two topics at once.
	// It chooses the two messages by minimizing the time difference between them
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image, sensor_msgs::Image> MyApproximateSynchronizationPolicy;

	//instantiate and set up the policy
	MyApproximateSynchronizationPolicy policy = MyApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	policy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	policy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	policy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more

	//Instantiate a Synchronizer with our policy.
	message_filters::Synchronizer<MyApproximateSynchronizationPolicy>  sync(MyApproximateSynchronizationPolicy(policy), arcoreCamera_sub, kinect_img_sub, kinect_depth_sub);

	//registers the callback
	auto f = boost::bind(&imagesCallback, _1, _2, _3, kinectCameraInfo);

	ROS_INFO_STREAM("waiting for images on topics: "<<endl<<
		ros::names::remap(input_arcore_topic)<<endl<<
		ros::names::remap(input_kinect_camera_topic)<<endl<<
		ros::names::remap(input_kinect_depth_topic)<<endl);
	sync.registerCallback(f);



	ros::spin();

	return 0;
}


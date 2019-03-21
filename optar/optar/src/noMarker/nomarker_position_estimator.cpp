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
#include <std_msgs/String.h>
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
#include "ARDeviceHandler.hpp"


using namespace cv;
using namespace std;


const string NODE_NAME 						= "nomarker_position_estimator";
const string input_arcore_topic				= "arcore_camera";
const string input_kinect_camera_topic		= "kinect_camera" ;
const string input_kinect_depth_topic		= "kinect_depth";
const string input_kinect_camera_info_topic	= "kinect_camera_info" ;
const string devices_heartbeats_topicName	= "heartbeats_topic" ;
const string debug_images_topic				= "debug_images_topic" ;

std::map<string, shared_ptr<ARDeviceHandler>> handlers;
std::shared_ptr<ros::NodeHandle> nodeHandle;


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

const unsigned int threadsNumber = 8;


void dynamicParametersCallback(optar::OptarDynamicParametersConfig &config, uint32_t level)
{
	ROS_INFO_STREAM("Reconfigure Request: "<<endl<<
            "pnp iterations = "<<config.pnp_iterations<<endl<<
            "pnp confidence = "<<config.pnp_confidence<<endl<<
            "pnp reporjection error = "<<config.pnp_reprojection_error<<endl<<
            "matching threshold ="<<config.matching_threshold<<endl<<
			"reporjection discard threshold =" <<config.reprojection_discard_threshold<<endl<<
			"orb max points = "<<config.orb_max_points<<endl<<
			"orb scale factor = "<<config.orb_scale_factor<<endl<<
			"orb levels number = "<<config.orb_levels_number<<endl<<
			"startup frames number = "<<config.startup_frames_num<<endl<<
			"phone orientation difference threshold = "<<config.phone_orientation_diff_thresh<<endl<<
			"show images = "<<config.show_images);



	pnpIterations 			= config.pnp_iterations;
	pnpReprojectionError 	= config.pnp_reprojection_error;
	pnpConfidence 			= config.pnp_confidence;

	matchingThreshold 					= config.matching_threshold;
	reprojectionErrorDiscardThreshold 	= config.reprojection_discard_threshold;

	orbMaxPoints		= config.orb_max_points;
	orbScaleFactor		= config.orb_scale_factor;
	orbLevelsNumber		= config.orb_levels_number;

	startupFramesNum						= config.startup_frames_num;
	phoneOrientationDifferenceThreshold_deg	= config.phone_orientation_diff_thresh;
	showImages								= config.show_images;

	for(auto const& keyValuePair: handlers)
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

void deviceHeartbeatsCallback(const std_msgs::StringConstPtr& msg)
{
	string deviceName = msg->data;

	auto it = handlers.find(deviceName);
	if(it==handlers.end())//if it dowsn't exist, create it
	{
		ROS_INFO_STREAM("New device detected, id="<<deviceName);
		shared_ptr<ARDeviceHandler> newHandler = make_shared<ARDeviceHandler>(deviceName, input_kinect_camera_topic, input_kinect_depth_topic, input_kinect_camera_info_topic, debug_images_topic);
		int r = newHandler->setupParameters(pnpReprojectionError,
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
		if(r<0)
		{
			ROS_ERROR_STREAM("Couldn't setup device handler parameters, error "<<r<<". Will not handle device "<<deviceName);
			return;
		}
		r = newHandler->start(nodeHandle);
		if(r<0)
		{
			ROS_ERROR_STREAM("Couldn't start device handler, error "<<r<<". Will not handle device "<<deviceName);
			return;
		}
		handlers.insert(std::map<string, shared_ptr<ARDeviceHandler>>::value_type(newHandler->getARDeviceId(),newHandler));
		ROS_INFO_STREAM("Started handler for device "<<newHandler->getARDeviceId());
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



	if(threadsNumber<2)
	{
		ROS_ERROR("threadsNumber has to be at least two.");
		return -1;
	}

	ros::Subscriber sub = nodeHandle->subscribe(devices_heartbeats_topicName, 10, deviceHeartbeatsCallback);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(devices_heartbeats_topicName));

	ros::MultiThreadedSpinner spinner(threadsNumber); // Use 4 threads
	spinner.spin(); // spin() will not return until the node has been shutdown
	ros::spin();

	return 0;
}


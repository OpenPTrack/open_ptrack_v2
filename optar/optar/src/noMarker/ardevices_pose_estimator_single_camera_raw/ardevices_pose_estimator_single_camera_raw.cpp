/**
 * @file
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 * Ros node that estimates the coordinate frame transformationn (registration) between
 * mobile cameras and the ros coordinate frame using one specific fixed camera.
 *
 * The estimation is based on common visual features between the two cameras
 *
 * New mobile cameras are detected listening at the heartbeats topic. Each mobile device
 * periodically sends an "heartbeat" message containing its deviceId. When this node
 * detects a new device it instantiates an ARDevicePoseEstimatorSingleCamera to listen for messages and
 * estimate its coordinate frame pose.
 * If no message is received from a device in a specific timeout period (handler_no_msg_timeout)
 * then the node removes the ARDevicePoseEstimatorSingleCamera for the device, and so stops listeneing for messages
 * from that specific device and deletes its registration estimation.
 *
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
#include <thread>         // this_thread::sleep_for
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>


#include <dynamic_reconfigure/server.h>
#include <optar/OptarSingleCameraParametersConfig.h>

#include "../../utils.hpp"
#include "ARDevicePoseEstimatorSingleCamera.hpp"
#include "FeaturesMemory.hpp"

using namespace cv;
using namespace std;

/** The ROS node name */
static const string NODE_NAME 						= "ardevices_pose_estimator_single_camera_raw";
/** topic for the fixed camera regular images */
static const string input_kinect_camera_topic		    = "kinect_camera" ;
/** topic for the fixed camera depth images */
static const string input_kinect_depth_topic		    = "kinect_depth";
/** topic fo the fixed camera CameraInfo */
static const string input_kinect_camera_info_topic	= "kinect_camera_info" ;
/** topic for the heartbeats from all the mobile cameras */
static const string devices_heartbeats_topicName	  = "heartbeats_topic" ;
/** topic to publish the raw registration estimation (unflitered and single-camera) */
static const string output_raw_pose_estimate_topic		  = "output_raw_pose_estimate_topic";
/** sensor name of the fixed camera */
static string fixed_sensor_name						          = "fixed_sensor_name";

/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg*/
static double pnpReprojectionError = 5;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double pnpConfidence = 0.99;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double pnpIterations = 1000;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double matchingThreshold = 25;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double reprojectionErrorDiscardThreshold = 5;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static int orbMaxPoints = 500;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double orbScaleFactor = 1.2;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static int orbLevelsNumber = 8;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static double phoneOrientationDifferenceThreshold_deg = 45;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static bool enableFeaturesMemory = false;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static bool showImages = false;
/** parameter controlled by dynamic_reconfigure, see  optar/cfg/OptarSingleCameraParameters.cfg */
static unsigned int minimumMatchesNumber = 4;
/** TODO: add parameter to dynamic_reconfigure. Timeout after which a device a device with no eartbeat is removed */
static int handler_no_msg_timeout = 5000;

static double maxPoseHeight = 2.5;
static double minPoseHeight = 0;

/** Number of threads that are used. Must be at least 2 to allow for message waiting in
    the callbacks. Estimations for different phones can be performed simultaneously */
const unsigned int threadsNumber = 8;





/** Handlers for the actove devices */
map<string, shared_ptr<ARDevicePoseEstimatorSingleCamera>> handlers;
/** Mutex for \link handlers \endlink */
timed_mutex handlersMutex;
/** Ros handle for this node */
shared_ptr<ros::NodeHandle> nodeHandle;
/** this keeps track of useful features to use later */
shared_ptr<FeaturesMemory> featuresMemory;

/**
 * Callback for dynamic_reconfigure parameters
 * @param config New values for the parameters
 * @param level  Not used
 */
void dynamicParametersCallback(optar::OptarSingleCameraParametersConfig &config, uint32_t level)
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
			"phone orientation difference threshold = "<<config.phone_orientation_diff_thresh<<endl<<
			"minimum matches number = "<<config.minimum_matches_number<<endl<<
			"show images = "<<config.show_images);



	pnpIterations 			= config.pnp_iterations;
	pnpReprojectionError 	= config.pnp_reprojection_error;
	pnpConfidence 			= config.pnp_confidence;

	matchingThreshold 					= config.matching_threshold;
	reprojectionErrorDiscardThreshold 	= config.reprojection_discard_threshold;

	orbMaxPoints		= config.orb_max_points;
	orbScaleFactor		= config.orb_scale_factor;
	orbLevelsNumber		= config.orb_levels_number;

	phoneOrientationDifferenceThreshold_deg	= config.phone_orientation_diff_thresh;

	minimumMatchesNumber = config.minimum_matches_number;

	showImages								= config.show_images;
	enableFeaturesMemory								= config.enable_features_memory;

	maxPoseHeight = config.max_pose_height;
	minPoseHeight = config.min_pose_height;

	//set the parameters for all the device handlers
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
						phoneOrientationDifferenceThreshold_deg,
						showImages,
						minimumMatchesNumber,
						enableFeaturesMemory,
					  maxPoseHeight,
					  minPoseHeight);

	}
}

/**
 * Callback fro the heatbeats from the mobile devices.
 * If it detects a new device it creates a new handler for it
 * @param msg Heartbeat message sent form a mobile device, it contains the deviceId
 */
void deviceHeartbeatsCallback(const std_msgs::StringConstPtr& msg)
{
	string deviceName = msg->data;

	unique_lock<timed_mutex> lock(handlersMutex, chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("deviceHeartbeatsCallback(): failed to get mutex. Skipping heartbeat for "<<deviceName);
		return;
	}

	ROS_DEBUG_STREAM("Received heartbeat from "<<msg->data);
	auto it = handlers.find(deviceName);
	if(it==handlers.end())//if it dowsn't exist, create it
	{
		ROS_INFO_STREAM("New device detected, id="<<deviceName);
		shared_ptr<ARDevicePoseEstimatorSingleCamera> newHandler = make_shared<ARDevicePoseEstimatorSingleCamera>(deviceName,
																			 input_kinect_camera_topic,
																			 input_kinect_depth_topic,
																			 input_kinect_camera_info_topic,
																			 fixed_sensor_name,
																			 output_raw_pose_estimate_topic,
																			 featuresMemory);
		int r = newHandler->setupParameters(pnpReprojectionError,
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
									enableFeaturesMemory,
								  maxPoseHeight,
									minPoseHeight);
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
		handlers.insert(map<string, shared_ptr<ARDevicePoseEstimatorSingleCamera>>::value_type(newHandler->getARDeviceId(),newHandler));
		ROS_INFO_STREAM("Started handler for device "<<newHandler->getARDeviceId());
	}
	else
	{
		it->second->signalDeviceAlive();
	}
}


/**
 * Remove handlers for devices for which we didn't received a message for a time
 * longer than handler_no_msg_timeout. It is called periodically
 */
void removeOldHandlers()
{
	unique_lock<timed_mutex> lock(handlersMutex, chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("removeOldHandlers(): failed to get mutex. Skipping.");
		return;
	}

	for (auto it = handlers.cbegin(); it != handlers.cend();)
	{
		int millisSinceMsg = it->second->millisecondsSinceLastMessage() ;

		if(millisSinceMsg>1500)
			ROS_INFO_STREAM(""<<it->first<< " no msg since "<<millisSinceMsg<< " ms");

		if (millisSinceMsg > handler_no_msg_timeout)
		{
			ROS_INFO_STREAM("removing handler for "<<it->first);
			it->second->stop();
			handlers.erase(it++);
		}
		else
		{
			//ROS_INFO_STREAM("keeping handler for "<<it->first<< " no msg since "<<millisSinceMsg<< " ms");
			++it;
		}
	}
}


/**
 * Main method for the node
 * @param  argc
 * @param  argv
 * @return
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	nodeHandle = make_shared<ros::NodeHandle>();
	ROS_INFO_STREAM("starting "<<NODE_NAME);

	featuresMemory = make_shared<FeaturesMemory>();

	fixed_sensor_name = ros::names::remap(fixed_sensor_name);
	//remove spaces and '/'
	string::iterator end_pos = remove(fixed_sensor_name.begin(), fixed_sensor_name.end(), ' ');
	fixed_sensor_name.erase(end_pos, fixed_sensor_name.end());
	if(fixed_sensor_name.at(0)=='/')
		fixed_sensor_name = fixed_sensor_name.substr(1);



	dynamic_reconfigure::Server<optar::OptarSingleCameraParametersConfig> server;
	dynamic_reconfigure::Server<optar::OptarSingleCameraParametersConfig>::CallbackType bindedDynamicParametersCallback;
	bindedDynamicParametersCallback = boost::bind(&dynamicParametersCallback, _1, _2);
	server.setCallback(bindedDynamicParametersCallback);



	if(threadsNumber<2)
	{
		ROS_ERROR("threadsNumber has to be at least two.");
		return -1;
	}

	ros::Subscriber sub = nodeHandle->subscribe(devices_heartbeats_topicName, 100, deviceHeartbeatsCallback);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(devices_heartbeats_topicName));

	ros::AsyncSpinner spinner(threadsNumber);
	spinner.start();

	ros::Rate loop_rate(0.5);

	while (ros::ok())
	{

		removeOldHandlers();

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

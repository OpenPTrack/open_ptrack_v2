/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * ARDeviceHandler class declaration file
 */

#ifndef AR_DEVICE_HANDLER_HPP_201903071552
#define AR_DEVICE_HANDLER_HPP_201903071552



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
#include <opencv2/core/eigen.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <mutex>

#include "../utils.hpp"
#include "ARDeviceHandler.hpp"

#include "ARDeviceRegistrationEstimator.hpp"
#include "FeaturesMemory.hpp"

/**
 * Handler for one AR device that estimates the coordinate frame registration
 * between the device and the ROS tf system using one fixed RGB-D camera.
 *
 * After construting an object you need to call the ARDeviceHandler::start() method,
 * this will start listening for messages from the device and so estimating
 * the transform between the two coordinate systems.
 *
 * The computed transform is published as a opt_msgs::ARDeviceRegistration message
 * on the topic specified in the constructor.
 * Actually, all the handlers for all the AR devices and all the fixed cameras publish
 * on the same topic, which is read from the ardevices_registration_aggregator.
 *
 * This class supports estimating the transform both using images from the AR device
 * (opt_msgs::ArcoreCameraImage messages) or from precomputed features sent from
 * the AR device (opt_msgs::ArcoreCameraFeatures messages)
 *
 * To perform the actual estimation it uses an ARDeviceRegistrationEstimator object
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 */
class ARDeviceHandler
{
private:
	std::shared_ptr<ARDeviceRegistrationEstimator> estimator;
	std::string ARDeviceId;

	std::string fixedCameraMonoTopicName;
	std::string fixedCameraDepthTopicName;
	std::string cameraInfoTopicName;
	std::string arDeviceCameraMsgTopicName;
	std::string arDeviceFeaturesMsgTopicName;
	std::string outputRawEstimationTopic;

	std::string fixed_sensor_name;

	geometry_msgs::TransformStamped transformKinectToWorld;
	sensor_msgs::CameraInfo cameraInfo;


	/** Synchronization policy for receiving the image message from the ar device
	    and also images from the fixed camera. It chooses the two messages by minimizing
			the time difference between them */
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image, sensor_msgs::Image> MyApproximateSynchronizationPolicy;
	std::shared_ptr<message_filters::Synchronizer<MyApproximateSynchronizationPolicy>> synchronizer;
	/** subscriber for receiving images, camera info and pose from the AR device */
	std::shared_ptr<message_filters::Subscriber<opt_msgs::ArcoreCameraImage>> arcoreCamera_sub;
	/** subscriber for receiving regular images from the fixed camera */
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_img_sub;
	/** subscriber for receiving depth images form the fixed camera */
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_depth_sub;


	/** Synchronization policy for receiving the feature message from the ar device
	    and also images from the fixed camera. It chooses the two messages by minimizing
			the time difference between them */
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraFeatures, sensor_msgs::Image, sensor_msgs::Image> FeaturesApproximateSynchronizationPolicy;
	std::shared_ptr<message_filters::Synchronizer<FeaturesApproximateSynchronizationPolicy>> featuresTpc_synchronizer;
	/** subscriber for receiving features, camera info and pose from the AR device */
	std::shared_ptr<message_filters::Subscriber<opt_msgs::ArcoreCameraFeatures>> featuresTpc_arcore_sub;
	/** subscriber for receiving regular images from the fixed camera */
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> featuresTpc_kinect_img_sub;
	/** subscriber for receiving depth images form the fixed camera */
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> featuresTpc_kinect_depth_sub;

	/** Publisher to publish the computed registration */
	ros::Publisher rawEstimationPublisher;




	/** Mutex to synchronize the methods of the ARDeviceHandler */
	std::timed_mutex objectMutex;
	/** To keep track of the time we last received a message */
 	std::chrono::steady_clock::time_point lastTimeReceivedMessage;
	/** stopped? */
 	bool stopped = false;

	/** To keep track of useful features and share them with the other handlers */
	std::shared_ptr<FeaturesMemory> featuresMemory;



	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double pnpReprojectionError = 5;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double pnpConfidence = 0.99;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double pnpIterations = 1000;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double matchingThreshold = 25;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double reprojectionErrorDiscardThreshold = 5;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	int orbMaxPoints = 500;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double orbScaleFactor = 1.2;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	int orbLevelsNumber = 8;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	double phoneOrientationDifferenceThreshold_deg = 45;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	bool showImages = true;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	unsigned int minimumMatchesNumber = 4;
	/** See ARDeviceRegistrationEstimator#setupParameters() */
	bool enableFeaturesMemory = false;

public:
	ARDeviceHandler(std::string ARDeviceId,
					 std::string fixedCameraMonoTopicName,
					 std::string fixedCameraDepthTopicName,
					 std::string cameraInfoTopicName,
					 std::string fixed_sensor_name,
					 std::string outputRawEstimationTopic,
					 std::shared_ptr<FeaturesMemory> featuresMemory);

	~ARDeviceHandler();
	int start(std::shared_ptr<ros::NodeHandle> NodeHandle);
	int stop();

	int setupParameters(double pnpReprojectionError,
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
						bool enableFeaturesMemory);

	std::string getARDeviceId();

	int millisecondsSinceLastMessage();

private:
	void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
						const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
						const sensor_msgs::ImageConstPtr& kinectInputDepthMsg);
	void featuresCallback(const opt_msgs::ArcoreCameraFeaturesConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg);
};

#endif

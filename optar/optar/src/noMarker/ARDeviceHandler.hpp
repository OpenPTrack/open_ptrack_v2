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



class ARDeviceHandler
{
private:
	std::shared_ptr<ARDeviceRegistrationEstimator> estimator;
	std::string ARDeviceId;

	std::string cameraRgbTopicName;
	std::string cameraDepthTopicName;
	std::string cameraInfoTopicName;
	std::string debugImagesTopic;
	std::string arDeviceCameraMsgTopicName;
	std::string arDeviceFeaturesMsgTopicName;
	std::string outputRawEstimationTopic;

	std::string fixed_sensor_name;

	geometry_msgs::TransformStamped transformKinectToWorld;
	sensor_msgs::CameraInfo cameraInfo;

	// Synchronization policy for having a callback that receives two topics at once.
	// It chooses the two messages by minimizing the time difference between them
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image, sensor_msgs::Image> MyApproximateSynchronizationPolicy;
	std::shared_ptr<message_filters::Synchronizer<MyApproximateSynchronizationPolicy>> synchronizer;
	std::shared_ptr<message_filters::Subscriber<opt_msgs::ArcoreCameraImage>> arcoreCamera_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_img_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_depth_sub;


	//subscribers and synchronizer for phone-side feature computation
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraFeatures, sensor_msgs::Image, sensor_msgs::Image> FeaturesApproximateSynchronizationPolicy;
	std::shared_ptr<message_filters::Synchronizer<FeaturesApproximateSynchronizationPolicy>> featuresTpc_synchronizer;
	std::shared_ptr<message_filters::Subscriber<opt_msgs::ArcoreCameraFeatures>> featuresTpc_arcore_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> featuresTpc_kinect_img_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> featuresTpc_kinect_depth_sub;

	ros::Publisher rawEstimationPublisher;

	double pnpReprojectionError = 5;
	double pnpConfidence = 0.99;
	double pnpIterations = 1000;
	double matchingThreshold = 25;
	double reprojectionErrorDiscardThreshold = 5;
	int orbMaxPoints = 500;
	double orbScaleFactor = 1.2;
	int orbLevelsNumber = 8;
	double phoneOrientationDifferenceThreshold_deg = 45;

	bool showImages = true;
	unsigned int minimumMatchesNumber = 4;


	std::timed_mutex objectMutex;

   	std::chrono::steady_clock::time_point lastTimeReceivedMessage;

   	bool stopped = false;

public:
	ARDeviceHandler(std::string ARDeviceId,
					 std::string cameraRgbTopicName,
					 std::string cameraDepthTopicName,
					 std::string cameraInfoTopicName,
					 std::string debugImagesTopic,
					 std::string fixed_sensor_name,
					 std::string outputRawEstimationTopic);

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
						unsigned int minimumMatchesNumber);

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
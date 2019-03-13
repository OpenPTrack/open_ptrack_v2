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

	geometry_msgs::TransformStamped transformKinectToWorld;
	sensor_msgs::CameraInfo cameraInfo;

	// Synchronization policy for having a callback that receives two topics at once.
	// It chooses the two messages by minimizing the time difference between them
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image, sensor_msgs::Image> MyApproximateSynchronizationPolicy;
	std::shared_ptr<message_filters::Synchronizer<MyApproximateSynchronizationPolicy>> synchronizer;
	std::shared_ptr<message_filters::Subscriber<opt_msgs::ArcoreCameraImage>> arcoreCamera_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_img_sub;
	std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> kinect_depth_sub;



	double pnpReprojectionError = 5;
	double pnpConfidence = 0.99;
	double pnpIterations = 1000;
	double matchingThreshold = 25;
	double reprojectionErrorDiscardThreshold = 5;
	int orbMaxPoints = 500;
	double orbScaleFactor = 1.2;
	int orbLevelsNumber = 8;
	unsigned int startupFramesNum = 10;
	double phoneOrientationDifferenceThreshold_deg = 45;
	double estimateDistanceThreshold_meters = 5;

	bool showImages = true;
	bool useCuda = true;

	std::timed_mutex objectMutex;

public:
	ARDeviceHandler(std::string ARDeviceId, std::string cameraRgbTopicName, std::string cameraDepthTopicName, std::string cameraInfoTopicName, std::string debugImagesTopic);
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
						unsigned int startupFramesNum,
						double phoneOrientationDifferenceThreshold_deg,
						double estimateDistanceThreshold_meters,
						bool showImages,
						bool useCuda);

	std::string getARDeviceId();

private:
	void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
						const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
						const sensor_msgs::ImageConstPtr& kinectInputDepthMsg);
};

#endif
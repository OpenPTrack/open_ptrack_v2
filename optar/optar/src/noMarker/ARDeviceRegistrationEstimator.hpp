#ifndef AR_DEVICE_REGISTRATION_ESIMATOR_HPP
#define AR_DEVICE_REGISTRATION_ESIMATOR_HPP

#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <opt_msgs/ArcoreCameraFeatures.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include  "TransformFilterKalman.hpp"

class ARDeviceRegistrationEstimator
{
private:
	std::vector<tf::Pose> arcoreWorldHistory;
	tf::Pose lastEstimate;
	bool didComputeEstimation=false;
	bool createdMatchesWindow = false;
	bool createdReprojectionWindow = false;


	std::string ARDeviceId;


	ros::Publisher pose_raw_pub;
	ros::Publisher pose_marker_pub;
	image_transport::Publisher matches_images_pub;
	image_transport::Publisher reproj_images_pub;
	geometry_msgs::TransformStamped transformKinectToWorld;
	std::shared_ptr<TransformFilterKalman> transformFilterKalman;

	const std::string namespaceName = "optar";
	const std::string outputPoseRaw_topicName			= "no_marker_pose_raw";
	const std::string outputPoseMarker_topicName		= "pose_marker";

	std::string matchesWindowName;
	std::string reprojectionWindowName;


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



public:

	ARDeviceRegistrationEstimator(std::string ARDeviceId, ros::NodeHandle& nh, geometry_msgs::TransformStamped transformKinectToWorld, std::string debugImagesTopic);



	void setupParameters(double pnpReprojectionError,
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


	int imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo);

	int featuresCallback(const opt_msgs::ArcoreCameraFeaturesConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo);

	int update(	const std::vector<cv::KeyPoint>& arcoreKeypoints,
				const cv::Mat& arcoreDescriptors,
				const std::vector<cv::KeyPoint>& fixedKeypoints,
				const cv::Mat& fixedDescriptors,
				const cv::Size& arcoreImageSize,
				const cv::Size& kinectImageSize,
				const cv::Mat& arcoreCameraMatrix,
				const cv::Mat& fixedCameraMatrix,
				cv::Mat& kinectDepthImage,
				const cv::Mat& kinectMonoImage,
				const cv::Mat& arcoreImage,
				const tf::Pose& phonePoseArcoreFrameConverted,
				const ros::Time& timestamp,
				const std::string fixedCameraFrameId);

	tf::Transform getEstimation();

	std::string getARDeviceId();

	bool hasEstimate();

private:



int computeOrbFeatures(const cv::Mat& image,
					std::vector<cv::KeyPoint>& keypoints,
					cv::Mat& descriptors);

int findOrbMatches(	const std::vector<cv::KeyPoint>& arcoreKeypoints,
					const cv::Mat& arcoreDescriptors,
					const std::vector<cv::KeyPoint>& kinectKeypoints,
					const cv::Mat& kinectDescriptors,
					std::vector<cv::DMatch>& matches);

int filterMatches(const std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& goodMatches);

int readReceivedImageMessages(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo,
					cv::Mat& arcoreCameraMatrix,
					cv::Mat& arcoreImg,
					cv::Mat& kinectCameraMatrix,
					cv::Mat& kinectCameraImg,
					cv::Mat& kinectDepthImg,
					tf::Pose& phonePoseArcoreFrameConverted);
int readReceivedMessages_features(const opt_msgs::ArcoreCameraFeaturesConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo,
					cv::Mat& arcoreCameraMatrix,
					cv::Mat& arcoreDescriptors,
					std::vector<cv::KeyPoint>& arcoreKeypoints,
					cv::Size& arcoreImageSize,
					cv::Mat& kinectCameraMatrix,
					cv::Mat& kinectCameraImg,
					cv::Mat& kinectDepthImg,
					tf::Pose& phonePoseArcoreFrameConverted,
					cv::Mat& debugArcoreImage);


/**
 * Checks the provided matches to ensure they have valid depth info. If the depth is not available in the depth image
 * this funciton will try to fix the image by getting the closest depth value. If the closest valid pixel is too far
 * the match will be dropped.
 */
int fixMatchesDepthOrDrop(const std::vector<cv::DMatch>& inputMatches, const std::vector<cv::KeyPoint>& kinectKeypoints, cv::Mat& kinectDepthImg,std::vector<cv::DMatch>& outputMatches);


int get3dPositionsAndImagePositions(const std::vector<cv::DMatch>& inputMatches,
	const std::vector<cv::KeyPoint>& kinectKeypoints,
	const std::vector<cv::KeyPoint>& arcoreKeypoints,
	const cv::Mat& kinectDepthImg,
	const cv::Mat& kinectCameraMatrix,
    std::vector<cv::Point3f>& matches3dPos,
    std::vector<cv::Point2f>& matchesImgPos);


void closeWindows();

bool detectAndFollowTransformJump();

static double getDistanceVariance(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex);

};



#endif

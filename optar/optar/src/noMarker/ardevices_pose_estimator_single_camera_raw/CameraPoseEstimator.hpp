/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * CameraPoseEstimator class declaration file
 */

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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

#include "FeaturesMemory.hpp"


/**
 * Class for estimating the pose of a mobile camera using features also seen from
 * a fixed depth camera
 */
class CameraPoseEstimator
{
private:

	/** Laste estimate of the phone pose */
	geometry_msgs::PoseStamped lastPoseEstimate;
	/** Number of matches used to compute the last estimate */
	int lastEstimateMatchesNumber = -1;
	/** Reprojection error in the last estimation */
	double lastEstimateReprojectionError = -1;
	/** If we ever computed an estimation */
	bool didComputeEstimate=false;

	/** DEvice id for the AR device for which we are estimating the registration */
	std::string ARDeviceId;

	/** Used to keep memory of useful features, and share them with other estimators */
	std::shared_ptr<FeaturesMemory> featuresMemory;

	/** for debug purpouses, used to publish the estimated pose of the phone */
	ros::Publisher pose_raw_pub;
	/** for debug purpouses, used to publish rviz markers for 3d points used to copute the estimation */
	ros::Publisher debug_markers_pub;
	/** for debug purpouses, used to publish an image showing the matches between mobile and fixed camera images */
	image_transport::Publisher matches_images_pub;
	/** for debug purpouses, used to publish an image showing the reprojection of the 3d points on the mobile camera image */
	image_transport::Publisher reproj_images_pub;
	/** the transformation between the fixed camera frame and the /world frame */
	geometry_msgs::TransformStamped transformFixedCameraToWorld;

	/** name of the namespace of the topics, used to build the output topics names */
	const std::string namespaceName = "optar";
	/** used to build the name of the  topic on which we output the DEBUG raw phone pose*/
	const std::string outputPoseRaw_topicName			= "no_marker_pose_raw";
	/** used to build the name of the  topic on which we output the rviz markers for the used 3d points*/
	const std::string outputPoseMarker_topicName		= "pose_marker";
	/** sensor name of the fixed camera */
	std::string fixed_sensor_name;


	/** See #setupParameters() */
	double pnpReprojectionError = 5;
	/** See #setupParameters() */
	double pnpConfidence = 0.99;
	/** See #setupParameters() */
	double pnpIterations = 1000;
	/** See #setupParameters() */
	double matchingThreshold = 25;
	/** See #setupParameters() */
	double reprojectionErrorDiscardThreshold = 5;
	/** See #setupParameters() */
	int orbMaxPoints = 500;
	/** See #setupParameters() */
	double orbScaleFactor = 1.2;
	/** See #setupParameters() */
	int orbLevelsNumber = 8;
	/** See #setupParameters() */
	double phoneOrientationDifferenceThreshold_deg = 45;
	/** See #setupParameters() */
	unsigned int minimumMatchesNumber = 4;

	/** See #setupParameters() */
	bool showImages = true;
	/** See #setupParameters() */
	bool enableFeaturesMemory = false;

	float keypointMinDistThreshold = 5;

	double maxPoseHeight = 2.5;
	double minPoseHeight = 0;
public:

	CameraPoseEstimator(	std::string ARDeviceId,
									ros::NodeHandle& nh,
									geometry_msgs::TransformStamped transformFixedCameraToWorld,
									std::string fixed_sensor_name,
									std::shared_ptr<FeaturesMemory> featuresMemory);



	void setupParameters(double pnpReprojectionError,
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
						bool enableFeaturesMemory,
					  double maxPoseHeight,
					  double minPoseHeight);

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
				const ros::Time& timestamp,
				const std::string fixedCameraFrameId);

	geometry_msgs::PoseStamped getLastPoseEstimate();

	std::string getARDeviceId();

	bool hasEstimate();

	int getLastEstimateMatchesNumber();

	double getLastEstimateReprojectionError();

private:



	int computeOrbFeatures(const cv::Mat& image,
						std::vector<cv::KeyPoint>& keypoints,
						cv::Mat& descriptors);

	int findOrbMatches(
						const cv::Mat& arcoreDescriptors,
						const cv::Mat& kinectDescriptors,
						std::vector<cv::DMatch>& matches);

	int filterMatches(const std::vector<cv::DMatch>& matches,
		 								std::vector<cv::DMatch>& goodMatches,
										const std::vector<cv::KeyPoint>& arcoreKeypoints,
										const std::vector<cv::KeyPoint>& fixedKeypoints);

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
						cv::Mat& debugArcoreImage);


	int fixMatchesDepthOrDrop(const std::vector<cv::DMatch>& inputMatches, const std::vector<cv::KeyPoint>& kinectKeypoints, cv::Mat& kinectDepthImg,std::vector<cv::DMatch>& outputMatches);


	int get3dPositionsAndImagePositions(const std::vector<cv::DMatch>& inputMatches,
		const std::vector<cv::KeyPoint>& kinectKeypoints,
		const std::vector<cv::KeyPoint>& arcoreKeypoints,
		const cv::Mat& kinectDepthImg,
		const cv::Mat& kinectCameraMatrix,
	    std::vector<cv::Point3f>& matches3dPos,
	    std::vector<cv::Point2f>& matchesImgPos);

	int computeMobileCameraPose(const cv::Mat& mobileCameraMatrix,
                              const std::vector<cv::Point3f>& matches3dPositions,
                              const std::vector<cv::Point2f>& matchesImgPixelPos,
                              std::vector<int>& inliers,
															geometry_msgs::Pose& resultPose);

  void drawAndSendReproectionImage(const cv::Mat& arcoreImage,
																const std::vector<int>& inliers,
																const std::vector<cv::Point2f>& matchesImgPixelPos,
																const std::vector<cv::Point2f>& reprojectedPoints);


	double computeReprojectionError(const geometry_msgs::Pose& pose,
														const std::vector<cv::Point3f>& points3d,
														const cv::Mat& mobileCameraMatrix,
														const std::vector<cv::Point2f>& points2d,
														const std::vector<int>& inliers,
														std::vector<cv::Point2f>& reprojectedPoints);

	double computeAngleFromZAxis(const geometry_msgs::Pose& pose);

	void saveInliersToMemory(const std::vector<int>& inliers,
		const std::vector<cv::Point3f>& goodMatches3dPos,
		const geometry_msgs::Pose& cameraPose_fixedCameraFrame,
		const std::vector<cv::DMatch>& goodMatches,
		const std::vector<cv::KeyPoint>& fixedKeypoints,
		const cv::Mat& fixedDescriptors,
		const cv::Mat& kinectDepthImage);

	void closeWindows();

	bool detectAndFollowTransformJump();

	static double getDistanceVariance(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex);

};



#endif

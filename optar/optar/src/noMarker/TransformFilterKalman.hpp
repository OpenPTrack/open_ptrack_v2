/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the TransformFilterKalman class
 */
#ifndef TRANSFORM_KALMAN_FILTER_HPP
#define TRANSFORM_KALMAN_FILTER_HPP


#include <tf/tf.h>
#include <opencv2/video/tracking.hpp>
#include "TransformFilter.hpp"


/**
 * Filter for 3d Poses that makes use of a Kalman filter and some heuristics
 */
class TransformFilterKalman : public TransformFilter
{
public:
	/** Number of states monitored by the Kalman Filter */
	static const int numberOfStates = 6;
	/** Number of measured states used by the Kalman Filter */
	static const int numberOfMeasurements = 6;
	/** Number of inputs (action controls) used by the Kalman Filter */
	static const int numberOfInputs = 0;

private:
	/** Process noise covariance value used to initialize the Kalman filter */
	double mProcessNoiseCovariance;
	/** Measurement noise covariance value used to initialize the Kalman filter */
	double mMeasurementNoiseCovariance;
	/** A posteriori error covariance value used to initialize the Kalman filter */
	double mPosterioriErrorCovariance;

	/** If the Kalman filter ever got updated */
	bool didKalmanEverUpdate = false;

	/** The OpenCV Kalman Filter */
	cv::KalmanFilter kalmanFilter;

	/** Pose history used by the transform jump detector */
	std::deque<tf::Transform> transformJumpDetectorHistory;
	/** Number of transforms used by the jump detector */
	size_t transformJumpDetectorLength = 10;
	/** Number of input poses used to initialize the Kalman Filter */
	unsigned int startupFramesNum = 10;
	/** Maximum distance between a new input pose and the current estimate for the new pose to be taken into consideration */
	double estimateDistanceThreshold_meters = 5;

	/** History of the input raw poses, used to initialize the Kalman filter */
	std::vector<tf::Pose> rawEstimationsHistory;
	/** The last computed estimate */
	tf::Pose lastEstimate;


	double		getDistanceVariance(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex, const tf::Vector3& positionMean);
	tf::Vector3 computePositionMean(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex);
	bool 		detectAndFollowTransformJump();
	tf::Pose	matToPose(const cv::Mat& state);
	cv::Mat 	poseToMat(const tf::Pose& pose);
	tf::Pose updateKalman(const tf::Pose& pose);

public:
	TransformFilterKalman(double processNoiseCovariance, double measurementNoiseCovariance, double posterioriErrorCovariance, int startupFramesNum, double estimateDistanceThreshold_meters);

	virtual tf::Pose update(const tf::Pose& pose) override;

	void forceState(const tf::Pose& pose);

	void setupParameters(int startupFramesNum, double estimateDistanceThreshold_meters);
};


#endif

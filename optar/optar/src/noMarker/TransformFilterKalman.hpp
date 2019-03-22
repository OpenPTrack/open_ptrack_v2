/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the TransformFilterKalman class, which implements a kalman filter for
 * 3D static poses
 */
#ifndef TRANSFORM_KALMAN_FILTER_HPP
#define TRANSFORM_KALMAN_FILTER_HPP


#include <tf/tf.h>
#include <opencv2/video/tracking.hpp>
#include "TransformFilter.hpp"


class TransformFilterKalman : public TransformFilter
{
public:
	static const int numberOfStates = 6;            // the number of states
	static const int numberOfMeasurements = 6;       // the number of measured states
	static const int numberOfInputs = 0;             // the number of action control

private:
	double mProcessNoiseCovariance;
	double mMeasurementNoiseCovariance;
	double mPosterioriErrorCovariance;

	bool didEverUpdate = false;

	cv::KalmanFilter kalmanFilter;

	std::deque<tf::Transform> transformJumpDetectorHistory;
	size_t transformJumpDetectorLength = 10;
	unsigned int startupFramesNum = 10;
	double estimateDistanceThreshold_meters = 5;

	std::vector<tf::Pose> rawEstimationsHistory;
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
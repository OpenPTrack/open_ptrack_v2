/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the TransformKalmanFilter class, which implements a kalman filter for
 * 3D static poses
 */
#ifndef TRANSFORM_KALMAN_FILTER_HPP
#define TRANSFORM_KALMAN_FILTER_HPP


#include <tf/tf.h>
#include <opencv2/video/tracking.hpp>


class TransformKalmanFilter
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

public:
	TransformKalmanFilter(double processNoiseCovariance, double measurementNoiseCovariance, double posterioriErrorCovariance);

	tf::Pose update(tf::Pose pose);

};


#endif
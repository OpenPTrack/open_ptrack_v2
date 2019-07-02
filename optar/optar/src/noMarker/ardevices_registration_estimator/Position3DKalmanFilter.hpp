
/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the Position3DKalmanFilter class
 */
#ifndef POSITION_3D_KALMAN_FILTER_HPP
#define POSITION_3D_KALMAN_FILTER_HPP


#include <tf/tf.h>
#include <opencv2/video/tracking.hpp>


/**
 * Class implementing a kalman filter for a 3D position. It models the position along with its first and second derivative
 */
class Position3DKalmanFilter
{
private:

	/** OpenCV Kalman Filter used to help with the computation */
	cv::KalmanFilter kalmanFilter;

	/** Number of state components */
	const unsigned int stateVectorSize = 9;
	/** Size of the measurement vector */
	const unsigned int measurementVectorSize = 3;

	double processNoiseVarianceFactor;

	bool mDidEverComputeState = false;

  cv::Mat processNoiseCovariance(double timestep);
	cv::Mat transitionMatrix(double timestep);
	tf::Vector3 lastStateEstimate;

public:
  Position3DKalmanFilter();
	void setupParameters( double measurementNoiseVariance, double processNoiseVarianceFactor);
	cv::Mat update(const cv::Mat& measurement, double timestep_sec);
	tf::Vector3 update(const tf::Vector3& measurement, double timestep_sec);
	bool didEverComputeState();
	tf::Vector3 getState();
};

#endif

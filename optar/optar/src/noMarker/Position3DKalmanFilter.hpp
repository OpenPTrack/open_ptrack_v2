
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



class Position3DKalmanFilter
{
private:

	/** OpenCV Kalman Filter used to help with the computation */
	cv::KalmanFilter kalmanFilter;

	/** Number of state components */
	const unsigned int stateVectorSize = 9;
	/** Size of the measurement vector */
	const unsigned int measurementVectorSize = 3;



  cv::Mat processNoiseCovariance(double timestep);
	cv::Mat transitionMatrix(double timestep);
	void setupParameters( double measurementNoiseVarianceX,
                        double measurementNoiseVarianceY,
                        double measurementNoiseVarianceZ);

public:
  Position3DKalmanFilter();
	cv::Mat update(cv::Mat measurement, double timestep_sec);
	tf::Vector3 update(tf::Vector3 measurement, double timestep_sec);
};

#endif

/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the ARDevicePoseFusionFilter class
 */
#ifndef ARDEVICE_POSE_FUSION_FILTER_HPP
#define ARDEVICE_POSE_FUSION_FILTER_HPP


#include <tf/tf.h>
#include <opencv2/video/tracking.hpp>



class ARDevicePoseFusionFilter
{
private:

	/** OpenCV Kalman Filter used to help with the computation */
	cv::KalmanFilter kalmanFilter;

	/** Number of state components */
	const unsigned int stateVectorSize = 16;
	/** Size of the measurement vector */
	const unsigned int measurementVectorSize = 7;

	/** covariance matrix for measurements coming from arcore */
	cv::Mat arcoreMeasurementNoiseCov;
	/** covariance matrix for measurements coming from PnP */
	cv::Mat pnpMeasurementNoiseCov;

  tf::Pose update(const tf::Pose measurement, const cv::Mat& sensorNoiseCovariance);

  /**
   * Computes the Jacobian of the transition/prediction function (i.e. f(x,u,0)) with respect to
   * the state (i.e. x), computed at the provided state.
   * @param  state State to compute the Jacobian at
   * @return       The Jacobian
   */
  cv::Mat transitionFunctionJacobian(const cv::Mat& state);

  /**
   * Computes the Jacobian of the measurement function (i.e. h(x,0)) with respect to
   * the state (i.e. x), computed at the provided state.
   * @param  state State to compute the Jacobian at
   * @return       The Jacobian
   */
  cv::Mat measurementJacobian(const cv::Mat& state);

  
  cv::Mat predictState(const cv::Mat& previousState, double timestep);

public:
  ARDevicePoseFusionFilter();
  tf::Pose updateArcore(const tf::Pose& measurement);
  tf::Pose updatePnP(const tf::Pose& measurement);
};

#endif

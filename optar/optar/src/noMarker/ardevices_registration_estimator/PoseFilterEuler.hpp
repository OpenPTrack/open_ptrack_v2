
/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Definition of the Position3DKalmanFilter class
 */
#ifndef POSE_FILTER_EULER_HPP
#define POSE_FILTER_EULER_HPP


#include <tf/tf.h>
#include "Position3DKalmanFilter.hpp"


/**
 * Class implementing a Kalman filter for a 3D pose.
 * Uses the Euler angles to represent the orientation.
 * Orientation and position are filtered independently using two Position3DKalmanFilter objects
 */
class PoseFilterEuler
{
private:

	/** Position filter */
	Position3DKalmanFilter positionFilter;
	/** Orientation filter */
  Position3DKalmanFilter orientationFilter;

	/** LAst filtered pose estimate */
  tf::Pose lastPoseEstimate;
	/** True if a filtered pose estimate was ever computed */
  bool mDidEverComputeEstimate=false;

	/** Variance multiplictive factor for the position process noise */
  double positionProcessVarianceFactor;
	/** Measurement variance for the position filter (the measurement noise is set to be a diagonal matrix with this value of the diagonal)*/
  double positionMeasurementVariance;
	/** Variance multiplicative factor for the orientation process noise */
  double orientationProcessVarianceFactor;
	/** Measurement variance for the orientation filter (the measurement noise is set to be a diagonal matrix with this value of the diagonal)*/
  double orientationMeasurementVariance;

public:
  void setupParameters(double positionProcessVarianceFactor, double positionMeasurementVariance,
                       double orientationProcessVarianceFactor, double orientationMeasurementVariance);
  PoseFilterEuler();
  tf::Pose update(const tf::Pose& measurement, double timestep_sec, double positionMeasurementVariance, double orientationMeasurementVariance);
  tf::Pose update(const tf::Pose& measurement, double timestep_sec);

	bool didEverComputeEstimate();
	tf::Pose getLastPoseEstimate();


};

#endif

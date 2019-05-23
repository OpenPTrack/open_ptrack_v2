
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



class PoseFilterEuler
{
private:

	Position3DKalmanFilter positionFilter;
  Position3DKalmanFilter orientationFilter;

  tf::Pose lastPoseEstimate;
  bool mDidEverComputeEstimate=false;

  double positionProcessVarianceFactor;
  double positionMeasurementVariance;
  double orientationProcessVarianceFactor;
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

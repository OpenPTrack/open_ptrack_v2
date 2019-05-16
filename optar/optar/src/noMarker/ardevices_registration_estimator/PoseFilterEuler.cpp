#include "PoseFilterEuler.hpp"

using namespace std;
using namespace cv;

PoseFilterEuler::PoseFilterEuler()
{
  //nothing to do
}

void PoseFilterEuler::setupParameters(double positionProcessVarianceFactor, double positionMeasurementVariance,
                     double orientationProcessVarianceFactor, double orientationMeasurementVariance)
{
  this->positionProcessVarianceFactor = positionProcessVarianceFactor;
  this->positionMeasurementVariance = positionMeasurementVariance;
  this->orientationProcessVarianceFactor = orientationProcessVarianceFactor;
  this->orientationMeasurementVariance = orientationMeasurementVariance;

  positionFilter.setupParameters(positionProcessVarianceFactor, positionMeasurementVariance);
  orientationFilter.setupParameters(orientationProcessVarianceFactor, orientationMeasurementVariance);
}

tf::Pose PoseFilterEuler::update(const tf::Pose& measurement, double timestep_sec)
{
  tf::Vector3 positionMeasurement = measurement.getOrigin();
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(measurement.getRotation());
  mat.getEulerYPR(yaw, pitch, roll);
  tf::Vector3 orientationYPR(yaw,pitch,roll);

  tf::Vector3 positionState = positionFilter.update(positionMeasurement,timestep_sec);
  tf::Vector3 orientationStateYPR = orientationFilter.update(orientationYPR,timestep_sec);

  tf::Quaternion orientationQuaternionState;
  orientationQuaternionState.setEuler(orientationStateYPR.x(),orientationStateYPR.y(), orientationStateYPR.z());
  tf::Pose poseState(orientationQuaternionState, positionState);
  lastPoseEstimate = poseState;
  didEverComputeEstimate = true;
  return lastPoseEstimate;
}

tf::Pose PoseFilterEuler::update(const tf::Pose& measurement, double timestep_sec, double positionMeasurementVariance, double orientationMeasurementVariance)
{
  setupParameters(positionProcessVarianceFactor, positionMeasurementVariance,orientationProcessVarianceFactor, orientationMeasurementVariance);
  return update(measurement, timestep_sec);
}

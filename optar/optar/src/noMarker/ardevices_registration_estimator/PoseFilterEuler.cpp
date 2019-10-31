#include "PoseFilterEuler.hpp"

using namespace std;
using namespace cv;

/**
 * Constructs the filter
 */
PoseFilterEuler::PoseFilterEuler()
{
  //nothing to do
}

/**
 * Sets up the filter parameters
 * @param positionProcessVarianceFactor    Multiplicative factor for the position process noise covariance
 * @param positionMeasurementVariance      Measurement variance of the single position measurement components
 * @param orientationProcessVarianceFactor Multiplicative factor for the orientation process noise covariance
 * @param orientationMeasurementVariance   Measurement variance of the single orientation measurement components
 */
void PoseFilterEuler::setupParameters(double positionProcessVarianceFactor, double positionMeasurementVariance,
                     double orientationProcessVarianceFactor, double orientationMeasurementVariance)
{
  this->positionProcessVarianceFactor = positionProcessVarianceFactor;
  this->positionMeasurementVariance = positionMeasurementVariance;
  this->orientationProcessVarianceFactor = orientationProcessVarianceFactor;
  this->orientationMeasurementVariance = orientationMeasurementVariance;

  positionFilter.setupParameters(positionMeasurementVariance, positionProcessVarianceFactor);
  orientationFilter.setupParameters(orientationMeasurementVariance, orientationProcessVarianceFactor);
}

/**
 * Updates the filter by predictiong and correcting
 * @param  measurement  The new pose measurement
 * @param  timestep_sec Time since the last update in seconds
 * @return              The new filtered pose estimate
 */
tf::Pose PoseFilterEuler::update(const tf::Pose& measurement, double timestep_sec)
{
  tf::Vector3 positionMeasurement = measurement.getOrigin();
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(measurement.getRotation());
  mat.getEulerYPR(yaw, pitch, roll);
  tf::Vector3 orientationYPR(yaw,pitch,roll);

  //ROS_INFO("Updating position filter");
  tf::Vector3 positionState = positionFilter.update(positionMeasurement,timestep_sec);
  //ROS_INFO("Updating orientation filter");
  //orientationYPR = orientationFilter.update(orientationYPR,timestep_sec);
  //ROS_INFO("Filter updated");

  tf::Quaternion orientationQuaternionState;
  mat.setIdentity();
  mat.setEulerYPR(orientationYPR.x(),orientationYPR.y(), orientationYPR.z());
  mat.getRotation(orientationQuaternionState);
  //ROS_INFO_STREAM("orientation0 = "<<measurement.getRotation().x()<<", "<<measurement.getRotation().y()<<", "<<measurement.getRotation().z()<<", "<<measurement.getRotation().w());
  //ROS_INFO_STREAM("orientation1 = "<<orientationQuaternionState.x()<<", "<<orientationQuaternionState.y()<<", "<<orientationQuaternionState.z()<<", "<<orientationQuaternionState.w());
  tf::Pose poseState(orientationQuaternionState, positionState);
  lastPoseEstimate = poseState;
  mDidEverComputeEstimate = true;
  return lastPoseEstimate;
}

/**
 * Updates the filter by predictiong and correcting
 * @param  measurement                    The new pose measurement
 * @param  timestep_sec                   Time since the last update in seconds
 * @param  positionMeasurementVariance    variance of the position measurement
 * @param  orientationMeasurementVariance variance of the orientation measurement
 * @return                                The new filtered pose estimate
 */
tf::Pose PoseFilterEuler::update(const tf::Pose& measurement, double timestep_sec, double positionMeasurementVariance, double orientationMeasurementVariance)
{
  setupParameters(positionProcessVarianceFactor, positionMeasurementVariance,orientationProcessVarianceFactor, orientationMeasurementVariance);
  return update(measurement, timestep_sec);
}

/**
 * Checks if a filtered pose estimate was ever computed
 * @return True if a filtered pose estimate was ever computed
 */
bool PoseFilterEuler::didEverComputeEstimate()
{
  return mDidEverComputeEstimate;
}

/**
 * Returns the last computed filtered pose estimate
 * @return The posee estimate
 * @throws logic_error if no estimate was ever computed
 */
tf::Pose PoseFilterEuler::getLastPoseEstimate()
{
  if(!didEverComputeEstimate())
    throw logic_error(""+string(__func__)+": called with didEverComputeEstimate==false");
  return lastPoseEstimate;
}

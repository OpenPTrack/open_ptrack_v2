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

tf::Pose PoseFilterEuler::update(const tf::Pose& measurement, double timestep_sec, double positionMeasurementVariance, double orientationMeasurementVariance)
{
  setupParameters(positionProcessVarianceFactor, positionMeasurementVariance,orientationProcessVarianceFactor, orientationMeasurementVariance);
  return update(measurement, timestep_sec);
}


bool PoseFilterEuler::didEverComputeEstimate()
{
  return mDidEverComputeEstimate;
}

tf::Pose PoseFilterEuler::getLastPoseEstimate()
{
  if(!didEverComputeEstimate())
    throw logic_error(""+string(__func__)+": called with didEverComputeEstimate==false");
  return lastPoseEstimate;
}

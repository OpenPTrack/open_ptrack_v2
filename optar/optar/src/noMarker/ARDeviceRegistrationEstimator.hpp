/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * ARDevicePoseEstimatorSingleCamera class declaration file
 */

#ifndef AR_DEVICE_REGISTRATION_ESTIMATOR_HPP
#define AR_DEVICE_REGISTRATION_ESTIMATOR_HPP

#include <tf/tf.h>
#include "PoseFilterEuler.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <deque>
#include <chrono>
#include <opt_msgs/ARDevicePoseEstimate.h>
#include <mutex>

class ARDeviceRegistrationEstimator
{
private:
  std::string arDeviceId;

  struct SavedArcorePose
  {
    geometry_msgs::PoseStamped pose;
    std::chrono::steady_clock::time_point receptionTime;
  };
  static std::timed_mutex queueMutex;
  std::deque<SavedArcorePose> arcorePosesQueue;
  unsigned int maxTimeInList_millis;

  PoseFilterEuler poseFilter;
  double pnpMeasurementVarianceFactor;
  double arcoreMeasurementVarianceFactor;
  double positionProcessVariance;
  double positionMeasurementVariance;
  double orientationProcessVariance;
  double orientationMeasurementVariance;

  bool didComputeEstimate = false;
  geometry_msgs::TransformStamped lastRegistrationEstimate;
  ros::Time lastFilteredPoseTime;

  void computeAndPublishRegistration(const tf::Pose& arcorePose, const tf::Pose& rosPose, const ros::Time& timestamp);

  void processArcoreQueueUntilSendTime(const ros::Time& time);
public:
  ARDeviceRegistrationEstimator(std::string arDeviceId, unsigned int maxTimeInList_millis);
  std::string getARDeviceId();
  void setupParameters(double positionProcessVariance, double positionMeasurementVariance,
                       double orientationProcessVariance, double orientationMeasurementVariance,
                       double pnpMeasurementVarianceFactor, double arcoreMeasurementVarianceFactor);


  void onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimate& poseEstimate);
  void onArcorePoseReceived(const geometry_msgs::PoseStamped& pose);
};

#endif

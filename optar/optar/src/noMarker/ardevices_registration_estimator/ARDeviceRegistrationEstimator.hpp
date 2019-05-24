/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * ARDeviceRegistrationEstimator class declaration file
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
  std::timed_mutex queueMutex;
  std::deque<SavedArcorePose> arcorePosesQueue;
  ros::Duration queueMaxMsgAge;

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
  bool didEverFilterPose;
  tf::Pose lastPoseEstimate;

  ros::Timer oldMsgsProcessorCallerTimer;

  void computeAndPublishRegistration(const tf::Pose& arcorePose, const tf::Pose& rosPose, const ros::Time& timestamp);

  void processArcoreQueueMsgsSentBeforeTime(const ros::Time& time);
  void processOldArcoreMsgs(const ros::TimerEvent&);

  tf::Pose filterPose(const tf::Pose& newPoseMeasurement, const ros::Time& timestamp, bool isARCore);

public:
  ARDeviceRegistrationEstimator(const std::string& arDeviceId, const ros::Duration& queueMaxMsgAge);
  std::string getARDeviceId();
  void setupParameters(double positionProcessVariance, double positionMeasurementVariance,
                       double orientationProcessVariance, double orientationMeasurementVariance,
                       double pnpMeasurementVarianceFactor, double arcoreMeasurementVarianceFactor);


  void onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimate& poseEstimate);
  void onArcorePoseReceived(const geometry_msgs::PoseStamped& pose);

  void start(std::shared_ptr<ros::NodeHandle> nodeHandle);
};

#endif

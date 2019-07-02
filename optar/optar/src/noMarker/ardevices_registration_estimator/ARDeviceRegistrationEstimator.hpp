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

/** Class that performs the registration estimation between an AR device and the ROS reference frame */
class ARDeviceRegistrationEstimator
{
private:
  /** uinque ID for the AR device */
  std::string arDeviceId;

  /**
   * Pose coupled with the time at which it was received
   */
  struct SavedArcorePose
  {
    /** pose */
    geometry_msgs::PoseStamped pose;
    /** Reception time */
    std::chrono::steady_clock::time_point receptionTime;
  };
  /** Mutex for the #arcorePosesQueue object */
  std::timed_mutex queueMutex;
  /** Queue for the received arcore poses */
  std::deque<SavedArcorePose> arcorePosesQueue;
  /** MAx allowed age for a pose in the #arcorePosesQueue queue */
  ros::Duration queueMaxMsgAge;

  /** Filter used for filtering the deive pose */
  PoseFilterEuler poseFilter;
  /** Multiplicative factor for the variance of the measurements coming from PnP estimates, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double pnpMeasurementVarianceFactor;
  /** Multiplicative factor for the variance of the measurements coming from ARCore estimates, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double arcoreMeasurementVarianceFactor;
  /** Mutiplicative factor for the position process noise covariance, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double positionProcessVariance;
  /** position measurement noise covariance, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double positionMeasurementVariance;
  /** Mutiplicative factor for the position process noise covariance, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double orientationProcessVariance;
  /** Orientation measurement noise covariance, see cfg/OptarRegistrationEstimatorParameters.cfg */
  double orientationMeasurementVariance;

  /** True if a registration estiate was ever computed */
  bool didComputeEstimate = false;
  /** Last registration estimate that has been computed */
  geometry_msgs::TransformStamped lastRegistrationEstimate;
  /** Time at which the last filtere pose estimate was produced */
  ros::Time lastFilteredPoseTime;
  /** True if a filtered pose estimate has ever been computed */
  bool didEverFilterPose;
  /** The last computed fitlered pose estimate */
  tf::Pose lastPoseEstimate;

  /** Time for periodically processing the messages in the queue */
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

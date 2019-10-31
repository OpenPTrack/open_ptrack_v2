
#include "ARDeviceRegistrationEstimator.hpp"
#include "../../utils.hpp"
#include <mutex>

using namespace std;

/**
 * Constructes the estimator
 * @param arDeviceId     unique id for the AR device
 * @param queueMaxMsgAge Max age for msgs in the queue
 */
ARDeviceRegistrationEstimator::ARDeviceRegistrationEstimator(const std::string& arDeviceId,
                                                             const ros::Duration& queueMaxMsgAge)
{
  this->arDeviceId = arDeviceId;
  this->queueMaxMsgAge = queueMaxMsgAge;

  poseFilter.setupParameters(1,1,1,1);
}

/**
 * [ARDeviceRegistrationEstimator::getARDeviceId description]
 * @return the id
 */
string ARDeviceRegistrationEstimator::getARDeviceId()
{
  return arDeviceId;
}

/**
 * Sets up the parameters
 * @param positionProcessVariance         See member variable documentation
 * @param positionMeasurementVariance     See member variable documentation
 * @param orientationProcessVariance      See member variable documentation
 * @param orientationMeasurementVariance  See member variable documentation
 * @param pnpMeasurementVarianceFactor    See member variable documentation
 * @param arcoreMeasurementVarianceFactor See member variable documentation
 */
void ARDeviceRegistrationEstimator::setupParameters(double positionProcessVariance, double positionMeasurementVariance,
                     double orientationProcessVariance, double orientationMeasurementVariance,
                     double pnpMeasurementVarianceFactor, double arcoreMeasurementVarianceFactor)
{
  this->positionProcessVariance = positionProcessVariance;
  this->positionMeasurementVariance = positionMeasurementVariance;
  this->orientationProcessVariance = orientationProcessVariance;
  this->orientationMeasurementVariance = orientationMeasurementVariance;
  this->pnpMeasurementVarianceFactor = pnpMeasurementVarianceFactor;
  this->arcoreMeasurementVarianceFactor = arcoreMeasurementVarianceFactor;

  poseFilter.setupParameters( positionProcessVariance,
                              positionMeasurementVariance,
                              orientationProcessVariance,
                              orientationMeasurementVariance);
}


/**
 * Computes the registration and publishes it using the two corresponding ARCore-ROS poses
 * @param arcorePose The ARCore pose
 * @param rosPose    The ROS pose
 * @param timestamp  The timestamp for the estimate
 */
void ARDeviceRegistrationEstimator::computeAndPublishRegistration(const tf::Pose& arcorePose, const tf::Pose& rosPose, const ros::Time& timestamp)
{
  //You don't belive me? Dim:
  // let phonePoseArcoreFrameConverted = Pa
  // let arcoreWorld = A
  // let phonePoseTf_world = Pr
  //
  // A*Pa*0 = Pr*0
  // so:
  // A*Pa = Pr
  // so:
  // A*Pa*Pa^-1 = Pr*Pa^-1
  // so:
  // A = Pr*Pa^-1

  tf::Pose arcoreWorld = rosPose * arcorePose.inverse();

  if(!isPoseValid(arcoreWorld))
  {
    ROS_WARN_STREAM("Dropping pose estimate as it is invalid");
    return;
  }

  tf::StampedTransform stampedTransform(arcoreWorld, timestamp, "/world", arDeviceId+"_world_filtered");
  geometry_msgs::TransformStamped geomTransf;
  tf::transformStampedTFToMsg(stampedTransform,geomTransf);

  publishTransformAsTfFrame(stampedTransform);
  publishTransformAsTfFrame(rosPose,arDeviceId+"_estimate_filtered","/world",timestamp);

  lastRegistrationEstimate = geomTransf;
  didComputeEstimate = true;
  //ROS_INFO_STREAM("Published filtered transform "<<poseToString(arcoreWorld));
}

/**
 * Processes the messages in the queue that are older than the specified time.
 * The messages are used to update the Pose estimate, but not the registration estimate
 * @param time The threshold time
 */
void ARDeviceRegistrationEstimator::processArcoreQueueMsgsSentBeforeTime(const ros::Time& time)
{

  std::unique_lock<std::timed_mutex> lock(queueMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting ("<<arDeviceId<<")");
		return;
	}

  ROS_DEBUG_STREAM("Processing msgs in queue until "<<time.sec<<" (queue size = "<<arcorePosesQueue.size()<<", didComputeEstimate="<<didComputeEstimate<<")");
  int c = 0;
  while(arcorePosesQueue.size() > 0 && arcorePosesQueue.front().pose.header.stamp < time)
  {
    geometry_msgs::PoseStamped pose_arcore = arcorePosesQueue.front().pose;
    //ROS_INFO_STREAM("Got front #"<<c);
    arcorePosesQueue.pop_front();
    //ROS_INFO("Removed front");
    if(didComputeEstimate)
    {
      geometry_msgs::PoseStamped pose_world;
      tf2::doTransform(pose_arcore,pose_world,lastRegistrationEstimate);
      tf::Pose pose_world_tf;
      poseMsgToTF(pose_world.pose,pose_world_tf);
      publishPoseAsTfFrame(pose_world,"pose_arcore_transformed");

      tf::Pose filtered_pose_world_tf = filterPose(pose_world_tf, pose_world.header.stamp, true);


      tf::Pose pose_arcore_tf;
      poseMsgToTF(pose_arcore.pose,pose_arcore_tf);
      //computeAndPublishRegistration(pose_arcore_tf,filtered_pose_world_tf,pose_arcore.header.stamp);
      //ROS_INFO("Published");
    }
    c++;
  }
  //ROS_INFO_STREAM("Processed "<<c<<" messages");
}

/**
 * Updates the kalman filter with the provided pose
 * @param  newPoseMeasurement The new pose measurement to use. It has to be in the /world frame
 * @param  isARCore           true if it is a pose coming from ARCore, false otherwise
 * @return                    The filtered pose
 */
tf::Pose ARDeviceRegistrationEstimator::filterPose(const tf::Pose& newPoseMeasurement, const ros::Time& timestamp, bool isARCore)
{

  //ROS_INFO_STREAM("filtering with pose "<<poseToString(newPoseMeasurement));
  double timeDiff = 1;
  if(didEverFilterPose)
    timeDiff = (timestamp - lastFilteredPoseTime).toSec();

  if(timeDiff<0)
  {
    ROS_WARN_STREAM("Received out of order measurement, skipping. ("<<(isARCore?"ARCore":"PnP")<<", timeDiff="<<timeDiff<<")");
    return lastPoseEstimate;
  }

  double positionVariance = positionMeasurementVariance;
  double orientationVariance = orientationMeasurementVariance;
  if(isARCore)
  {
    positionVariance *= arcoreMeasurementVarianceFactor;
    orientationVariance *= arcoreMeasurementVarianceFactor;
  }
  else
  {
    positionVariance *= pnpMeasurementVarianceFactor;
    orientationVariance *= pnpMeasurementVarianceFactor;
  }



  tf::Pose filteredPose = poseFilter.update(newPoseMeasurement,
                          timeDiff,
                          positionVariance,
                          orientationVariance);
  //filteredPose.setRotation(newPoseMeasurement.getRotation());//bypass orientation filtering
  //ROS_INFO_STREAM("Filtered, pose = "<<poseToString(filteredPose));
  lastFilteredPoseTime = timestamp;
  lastPoseEstimate = filteredPose;
  didEverFilterPose=true;
  return filteredPose;
}

/**
 * Called when a new PnP pose estimate is received. It processes the ARCore poses that are older than
 * this one and then computes the registration estimate.
 * @param poseEstimate The new PnP pose estimate
 */
void ARDeviceRegistrationEstimator::onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimate& poseEstimate)
{
  //update the filter with the arcor estimates up to now
  processArcoreQueueMsgsSentBeforeTime(poseEstimate.cameraPose.header.stamp);


  tf::Pose pose_world_tf;
  poseMsgToTF(poseEstimate.cameraPose.pose,pose_world_tf);
  tf::Pose pose_arcore_tf = convertCameraPoseArcoreToRos(poseEstimate.cameraPose_mobileFrame);
  if(!isPoseValid(pose_world_tf))
  {
    ROS_WARN_STREAM(""<<__func__<<": Skipping, received invalid ROS pose "<<poseToString(pose_world_tf));
    return;
  }
  if(!isPoseValid(pose_arcore_tf))
  {
    ROS_WARN_STREAM(""<<__func__<<": Skipping, received invalid ARCore pose "<<poseToString(pose_arcore_tf));
    return;
  }


  tf::Pose pose_world_tf_filtered = filterPose(pose_world_tf,poseEstimate.cameraPose.header.stamp,false);

  publishTransformAsTfFrame(pose_arcore_tf, arDeviceId+"_arcore", "/world", poseEstimate.header.stamp);

  computeAndPublishRegistration(pose_arcore_tf, pose_world_tf_filtered, poseEstimate.cameraPose.header.stamp);
}

/**
 * Called when a new ARCore pose estimate is received. The new pose is stored in the queue to be processed later
 * @param pose the newly receive pose estimate
 */
void ARDeviceRegistrationEstimator::onArcorePoseReceived(const geometry_msgs::PoseStamped& pose)
{

  tf::Pose convertedPose_tf = convertCameraPoseArcoreToRos(pose.pose);
  geometry_msgs::PoseStamped convertedPose = pose;
  tf::poseTFToMsg(convertedPose_tf,convertedPose.pose);

  SavedArcorePose sap;
  sap.pose = convertedPose;
  sap.receptionTime = std::chrono::steady_clock::now();

  std::unique_lock<std::timed_mutex> lock(queueMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting ("<<arDeviceId<<")");
		return;
	}
  tf::Pose pose_arcore_tf;
  poseMsgToTF(pose.pose,pose_arcore_tf);
  if(!isPoseValid(pose_arcore_tf))
  {
    ROS_WARN_STREAM(""<<__func__<<": Skipping, received invalid ARCore pose "<<poseToString(pose_arcore_tf));
    return;
  }
  //ROS_INFO_STREAM("Received valid ARCore pose "<<poseToString(pose_arcore_tf));
  publishTransformAsTfFrame(convertedPose_tf, arDeviceId+"_arcore", "/world", pose.header.stamp);

  arcorePosesQueue.push_back(sap);
}

/**
 * Periodic update method.
 * It processes the ARCore poses that are too old to be useful as matches to PnP estimates
 * @param [name] [description]
 */
void ARDeviceRegistrationEstimator::processOldArcoreMsgs(const ros::TimerEvent&)
{
  processArcoreQueueMsgsSentBeforeTime(ros::Time::now()-queueMaxMsgAge);
}

/**
 * Starts up the estimator.
 * Starts up the periodic update method.
 * @param nodeHandle Current ROS node handle
 */
void ARDeviceRegistrationEstimator::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
  oldMsgsProcessorCallerTimer = nodeHandle->createTimer(queueMaxMsgAge*0.5,
                                                        &ARDeviceRegistrationEstimator::processOldArcoreMsgs,
                                                        this);
}

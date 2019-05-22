
#include "ARDeviceRegistrationEstimator.hpp"
#include "../../utils.hpp"
#include <mutex>

using namespace std;

ARDeviceRegistrationEstimator::ARDeviceRegistrationEstimator(const std::string& arDeviceId,
                                                             const ros::Duration& queueMaxMsgAge)
{
  this->arDeviceId = arDeviceId;
  this->queueMaxMsgAge = queueMaxMsgAge;

  poseFilter.setupParameters(1,1,1,1);
}

string ARDeviceRegistrationEstimator::getARDeviceId()
{
  return arDeviceId;
}

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

  tf::Pose arcoreWorld = rosPose * convertCameraPoseArcoreToRos(arcorePose).inverse();

  if(!isPoseValid(arcoreWorld))
  {
    ROS_WARN_STREAM("Dropping pose estimate as it is invalid");
    return;
  }

  publishTransformAsTfFrame(arcoreWorld,arDeviceId+"_world_filtered","/world",timestamp);
  ROS_INFO_STREAM("Published filtered transform "<<poseToString(arcoreWorld));
}


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
    geometry_msgs::PoseStamped pose = arcorePosesQueue.front().pose;
    //ROS_INFO_STREAM("Got front #"<<c);
    arcorePosesQueue.pop_front();
    //ROS_INFO("Removed front");
    if(didComputeEstimate)
    {
      geometry_msgs::PoseStamped pose_world;
      tf2::doTransform(pose,pose_world,lastRegistrationEstimate);
      tf::Pose pose_world_tf;
  		poseMsgToTF(pose_world.pose,pose_world_tf);

      double timeDiff = (pose.header.stamp - lastFilteredPoseTime).toSec();
      //ROS_INFO("filtering...");
      tf::Pose filteredPose = poseFilter.update(pose_world_tf,
                              timeDiff,
                              positionMeasurementVariance*arcoreMeasurementVarianceFactor,
                              orientationMeasurementVariance*arcoreMeasurementVarianceFactor);
      //ROS_INFO("Filtered");
      lastFilteredPoseTime = pose.header.stamp;
      computeAndPublishRegistration(pose_world_tf,filteredPose,pose.header.stamp);
      //ROS_INFO("Published");
    }
    c++;
  }
  //ROS_INFO_STREAM("Processed "<<c<<" messages");
}


void ARDeviceRegistrationEstimator::onPnPPoseReceived(const opt_msgs::ARDevicePoseEstimate& poseEstimate)
{
  //update the filter with the arcor estimates up to now
  processArcoreQueueMsgsSentBeforeTime(poseEstimate.header.stamp);


  tf::Pose pose_world_tf;
  poseMsgToTF(poseEstimate.cameraPose.pose,pose_world_tf);
  tf::Pose pose_arcore_tf;
  poseMsgToTF(poseEstimate.cameraPose_mobileFrame,pose_arcore_tf);
  computeAndPublishRegistration(pose_world_tf, pose_arcore_tf, poseEstimate.cameraPose.header.stamp);
}

void ARDeviceRegistrationEstimator::onArcorePoseReceived(const geometry_msgs::PoseStamped& pose)
{
  SavedArcorePose sap;
  sap.pose = pose;
  sap.receptionTime = std::chrono::steady_clock::now();

  std::unique_lock<std::timed_mutex> lock(queueMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Aborting ("<<arDeviceId<<")");
		return;
	}
  arcorePosesQueue.push_back(sap);
}

void ARDeviceRegistrationEstimator::processOldArcoreMsgs(const ros::TimerEvent&)
{
  processArcoreQueueMsgsSentBeforeTime(ros::Time::now()-queueMaxMsgAge);
}

void ARDeviceRegistrationEstimator::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
  oldMsgsProcessorCallerTimer = nodeHandle->createTimer(queueMaxMsgAge*0.5,
                                                        &ARDeviceRegistrationEstimator::processOldArcoreMsgs,
                                                        this);
}

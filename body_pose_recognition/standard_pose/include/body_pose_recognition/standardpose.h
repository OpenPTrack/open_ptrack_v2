#ifndef OPEN_PTRACK_BPR_STANDARD_POSE_
#define OPEN_PTRACK_BPR_STANDARD_POSE_

#include <ros/ros.h>
#include <opt_msgs/StandardSkeletonTrackArray.h>
#include <body_pose_estimation/skeleton_base.h>
#include <eigen3/Eigen/Eigen>

namespace open_ptrack
{
namespace bpr
{

typedef open_ptrack::bpe::SkeletonBase  Skeleton;

class StandardPose
{
public:
  StandardPose(const opt_msgs::SkeletonTrack& skel_msg);

  inline double
  getOrientationAngle() const { return m_orientation_angle; }

  inline Eigen::Vector3d
  getOrientationVector() const { return m_orientation_v; }

  inline Skeleton
  getSkeleton() const { return m_standard_pose_skeleton; }

  opt_msgs::StandardSkeletonTrack
  getStandardTrackMsg() const;

private:
  double m_orientation_angle;
  Eigen::Vector3d m_orientation_v;
  const Skeleton m_skeleton_base;
  Skeleton m_standard_pose_skeleton;
  const opt_msgs::SkeletonTrack m_skeleton_track;

  void rototranslate();

  void computeOrientation();
};
} // bpr
} // open_ptrack


#endif

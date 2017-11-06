#include <body_pose_estimation/skeleton_base.h>

namespace open_ptrack
{
namespace bpe
{

#if LINKS_STYLE == 0 // CHEST_CENTERED
const std::array<std::string, SkeletonJoints::SIZE>
SkeletonBase::JOINT_NAMES{ {
 "HEAD",
 "NECK",
 "RIGHT_SHOULDER",
 "RIGHT_ELBOW",
 "RIGHT_WRIST",
 "LEFT_SHOULDER",
 "LEFT_ELBOW",
 "LEFT_WRIST",
 "RIGHT_HIP",
 "RIGHT_KNEE",
 "RIGHT_ANKLE",
 "LEFT_HIP",
 "LEFT_KNEE",
 "LEFT_ANKLE",
 "CHEST"
} };
const std::array<std::pair<SkeletonJoints, SkeletonJoints>, SkeletonLinks::SIZE>
SkeletonLinks::LINKS =
{
  std::pair<SkeletonJoints, SkeletonJoints> (CHEST, NECK),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, RSHOULDER),
  std::pair<SkeletonJoints, SkeletonJoints> (RSHOULDER, RELBOW),
  std::pair<SkeletonJoints, SkeletonJoints> (RELBOW, RWRIST),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, LSHOULDER),
  std::pair<SkeletonJoints, SkeletonJoints> (LSHOULDER, LELBOW),
  std::pair<SkeletonJoints, SkeletonJoints> (LELBOW, LWRIST),
  std::pair<SkeletonJoints, SkeletonJoints> (CHEST, RHIP),
  std::pair<SkeletonJoints, SkeletonJoints> (RHIP, RKNEE),
  std::pair<SkeletonJoints, SkeletonJoints> (RKNEE, RANKLE),
  std::pair<SkeletonJoints, SkeletonJoints> (CHEST, LHIP),
  std::pair<SkeletonJoints, SkeletonJoints> (LHIP, LKNEE),
  std::pair<SkeletonJoints, SkeletonJoints> (LKNEE, LANKLE),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, HEAD)
};
#else // MPII
const std::vector<std::pair<SkeletonJoints, SkeletonJoints>>
SkeletonLinks::links =
{
  std::pair<SkeletonJoints, SkeletonJoints> (HEAD, NECK),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, RSHOULDER),
  std::pair<SkeletonJoints, SkeletonJoints> (RSHOULDER, RELBOW),
  std::pair<SkeletonJoints, SkeletonJoints> (RELBOW, RWRIST),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, LSHOULDER),
  std::pair<SkeletonJoints, SkeletonJoints> (LSHOULDER, LELBOW),
  std::pair<SkeletonJoints, SkeletonJoints> (LELBOW, LWRIST),
  std::pair<SkeletonJoints, SkeletonJoints> (CHEST, RHIP),
  std::pair<SkeletonJoints, SkeletonJoints> (RHIP, RKNEE),
  std::pair<SkeletonJoints, SkeletonJoints> (RKNEE, RANKLE),
  std::pair<SkeletonJoints, SkeletonJoints> (CHEST, LHIP),
  std::pair<SkeletonJoints, SkeletonJoints> (LHIP, LKNEE),
  std::pair<SkeletonJoints, SkeletonJoints> (LKNEE, LANKLE),
  std::pair<SkeletonJoints, SkeletonJoints> (NECK, CHEST)
};
#endif
SkeletonBase::SkeletonBase(const opt_msgs::SkeletonTrack& skel_msg):
  m_is_mutable(false)
{
  for (size_t i = 0; i < SkeletonJoints::SIZE; ++i)

  {
    m_joints.col(i) << skel_msg.joints[i].x, skel_msg.joints[i].y,
        skel_msg.joints[i].z;
  }
}
SkeletonBase::SkeletonBase():
  m_is_mutable(true)
{
  m_joints.fill(std::numeric_limits<double>::quiet_NaN());
}

} // bpe
} // open_ptrack

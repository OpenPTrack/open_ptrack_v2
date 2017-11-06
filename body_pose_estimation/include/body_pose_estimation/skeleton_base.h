#ifndef __BPE__SKELETON_BASE_H_
#define __BPE__SKELETON_BASE_H_

#include <vector>
#include <array>
#include <eigen3/Eigen/Eigen>
#include <opt_msgs/SkeletonTrack.h>

#define LINKS_STYLE 0 // 0 = CHEST_CENTERED, 1 = MPII

namespace open_ptrack
{
namespace bpe
{
enum SkeletonJoints
{
  HEAD = 0,
  NECK,
  RSHOULDER,
  RELBOW,
  RWRIST,
  LSHOULDER,
  LELBOW,
  LWRIST,
  RHIP,
  RKNEE,
  RANKLE,
  LHIP,
  LKNEE,
  LANKLE,
  CHEST,
  SIZE
};
class SkeletonLinks
{
public:
  static const size_t SIZE = 14;
  static const
  std::array<std::pair<SkeletonJoints, SkeletonJoints>, SkeletonLinks::SIZE>
  LINKS;
};

class SkeletonBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    Eigen::Matrix<double, 3, SkeletonJoints::SIZE> m_joints;
  bool m_is_mutable;
public:
  SkeletonBase(const opt_msgs::SkeletonTrack &skel_msg);
  SkeletonBase();

  static const std::array<std::string, SkeletonJoints::SIZE> JOINT_NAMES;

  inline Eigen::Matrix<double, 3, open_ptrack::bpe::SkeletonJoints::SIZE>
  getJoints() const
  {
    return m_joints;
  }
  inline Eigen::Vector3d
  getJoint(size_t i) const{ return m_joints.col(i);}

  inline void
  setJoint(size_t i, const Eigen::Vector3d& j)
  {
    if(not m_is_mutable)
      throw std::runtime_error("This SkeletonBase instance is not mutable");
    m_joints.col(i) << j;
  }
  inline void
  setJoints(const Eigen::Matrix<double, 3, SkeletonJoints::SIZE>& m)
  {
    if(not m_is_mutable)
      throw std::runtime_error("This SkeletonBase instance is not mutable");
    m_joints = m;
  }
  inline bool
  isValid(size_t i) const
  {
    return not m_joints.col(i).hasNaN();
  }
  inline bool
  isValid() const
  {
    return not m_joints.hasNaN();
  }
};

} // bpe
} // open_ptrack
#endif

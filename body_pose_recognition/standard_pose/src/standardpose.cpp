#include <body_pose_recognition/standardpose.h>

using namespace open_ptrack::bpe;

namespace open_ptrack{
namespace bpr
{

StandardPose::StandardPose(const opt_msgs::SkeletonTrack &skel_msg):
  m_skeleton_base(skel_msg),
  m_orientation_angle(std::numeric_limits<double>::quiet_NaN()),
  m_skeleton_track(skel_msg)
{
  m_orientation_v =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  m_standard_pose_skeleton.setJoints(
        Eigen::Matrix<double, 3, SkeletonJoints::SIZE>::Constant(
          3, SkeletonJoints::SIZE, std::numeric_limits<double>::quiet_NaN()));
  rototranslate();
}

void StandardPose::rototranslate()
{
  if(not m_skeleton_base.isValid(SkeletonJoints::CHEST))
  {
    //    throw std::runtime_error("CHEST not valid. I cannot compute standard pose");
    return;
  }
  // Translation
  const Eigen::Vector3d chest =
      m_skeleton_base.getJoint(SkeletonJoints::CHEST);
  Eigen::Matrix<double, 3, SkeletonJoints::SIZE> offset;
  for(size_t i = 0; i < SkeletonJoints::SIZE; ++i)
    offset.col(i) = - chest;
  m_standard_pose_skeleton.setJoints
      (m_skeleton_base.getJoints() + offset);

  // skeleton links normalization and redrawing
  Eigen::Matrix<double, 3, SkeletonJoints::SIZE> tmp_vectors =
      m_standard_pose_skeleton.getJoints();
  for(size_t i = 0; i < SkeletonLinks::SIZE; ++i)
  {
    const Eigen::Vector3d& j1 =
        tmp_vectors.col(SkeletonLinks::LINKS[i].first);
    const Eigen::Vector3d& j2 =
        tmp_vectors.col(SkeletonLinks::LINKS[i].second);
    const Eigen::Vector3d& unit_dir_vector = (j2 - j1).normalized();
    m_standard_pose_skeleton.setJoint(
          SkeletonLinks::LINKS[i].second,
          m_standard_pose_skeleton.getJoint(SkeletonLinks::LINKS[i].first)
          + unit_dir_vector);
  }

  // rotation
  computeOrientation();
  if ((m_orientation_v[0] > 0.0 and m_orientation_v[1] > 0)
      or (m_orientation_v[0] < 0 and m_orientation_v[1] > 0))
    m_orientation_angle = - m_orientation_angle;
  Eigen::Matrix3d rotation_matrix =
      Eigen::AngleAxisd(m_orientation_angle,
                        Eigen::Vector3d::UnitZ()).toRotationMatrix();
  for(size_t i = 0; i < SkeletonJoints::SIZE; ++i)
  {
    m_standard_pose_skeleton.setJoint(
          i, rotation_matrix * m_standard_pose_skeleton.getJoint(i));
  }
}

void StandardPose::computeOrientation()
{
  Eigen::Vector3d rshoulder =
      m_skeleton_base.getJoint(SkeletonJoints::RSHOULDER);
  Eigen::Vector3d lshoulder =
      m_skeleton_base.getJoint(SkeletonJoints::LSHOULDER);
  Eigen::Vector3d chest = m_skeleton_base.getJoint(SkeletonJoints::CHEST);
  Eigen::Vector3d rhip = m_skeleton_base.getJoint(SkeletonJoints::RHIP);
  Eigen::Vector3d lhip = m_skeleton_base.getJoint(SkeletonJoints::LHIP);
  // if valid shoulder -> check chest
  if (not (rshoulder.hasNaN() or lshoulder.hasNaN() or chest.hasNaN()))
  {

    m_orientation_v =
        ((chest - lshoulder).cross(chest - rshoulder)).normalized();
  }

  // if valid orientation vector -> calculate angle of the proj vector
  if (not m_orientation_v.hasNaN())
  {
    Eigen::Vector2d proj_orientation = m_orientation_v.head<2>();
    m_orientation_angle = acos(proj_orientation.dot(Eigen::Vector2d::UnitX()));
    //if (m_orientation_angle < 0) m_orientation_angle += 2 * M_PI;
  }
  else
  {
    // throw std::runtime_error("Orientation angle is NaN, check the skeleton you passed me");
  }

}

opt_msgs::StandardSkeletonTrack
StandardPose::getStandardTrackMsg() const
{
  opt_msgs::StandardSkeletonTrack sk;
  sk.age = m_skeleton_track.age;
  sk.box_2D = m_skeleton_track.box_2D;
  sk.confidence = 0.0;
  sk.distance = m_skeleton_base.getJoints().col(SkeletonJoints::CHEST).norm(); // to keep track of distance wrt ref
  sk.height = 0.0;
  sk.id = m_skeleton_track.id;
  sk.orientation = m_orientation_angle;
  sk.joints.reserve(SkeletonJoints::SIZE);
  const Eigen::Matrix<double, 3, SkeletonJoints::SIZE>& joints =
      m_standard_pose_skeleton.getJoints();
  for(uint i = 0; i < SkeletonJoints::SIZE; ++i)
  {
    const Eigen::Vector3d& jj = joints.col(i);
    opt_msgs::Track3D j;
    j.id = i;
    j.x = jj[0];
    j.y = jj[1];
    j.z = jj[2];
    sk.joints.push_back(j);
  }
  return sk;
}

} // bpr
} // open_ptrack

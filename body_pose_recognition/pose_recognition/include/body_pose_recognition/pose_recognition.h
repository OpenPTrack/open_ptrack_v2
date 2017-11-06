#ifndef _OPEN_PTRACK__POSE_RECOGNITION_H
#define _OPEN_PTRACK__POSE_RECOGNITION_H

#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opt_msgs/SkeletonTrackArray.h>
#include <body_pose_recognition/standardpose.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/filesystem.hpp>
#include <numeric>
#include <opt_msgs/PoseRecognitionArray.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

namespace open_ptrack
{
namespace bpr
{

class FileRow
{
public:
  std::string const& operator[](std::size_t index) const
  {
    return m_data[index];
  }
  std::size_t size() const
  {
    return m_data.size();
  }
  void readNextRow(std::istream& str)
  {
    std::string         line;
    std::getline(str, line);

    std::stringstream   lineStream(line);
    std::string         cell;

    m_data.clear();
    while(std::getline(lineStream, cell, m_separator))
    {
      m_data.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
      // If there was a trailing comma then add an empty element.
      m_data.push_back("");
    }
  }
  FileRow(const char separator):
    m_separator(separator)
  {}

private:
  std::vector<std::string>    m_data;
  const char                  m_separator;
};





class PoseRecognition{

  typedef open_ptrack::bpr::StandardPose StandardPose;
  typedef Eigen::Matrix<double, 3,
  open_ptrack::bpe::SkeletonJoints::SIZE> SkeletonMatrix;
public:

  PoseRecognition();

private:

  ros::NodeHandle m_nh;
  ros::NodeHandle m_private_nh;
  message_filters::Subscriber<opt_msgs::StandardSkeletonTrackArray> m_st_sk_sub;
  message_filters::Subscriber<opt_msgs::SkeletonTrackArray> m_sk_sub;
  message_filters::TimeSynchronizer<opt_msgs::StandardSkeletonTrackArray,
  opt_msgs::SkeletonTrackArray> m_sync;
  std::map<size_t, std::vector<SkeletonMatrix>> m_gallery_poses;
  std::map<size_t, std::string> m_gallery_poses_names;
  std::map<size_t, std::vector<double>> m_per_frame_scores;
  std::map<size_t, double> m_final_scores;
  bool m_use_right_leg, m_use_right_arm, m_use_left_leg, m_use_left_arm;
  // how to fuse scores from the different body parts in a unique one
  uint m_per_skeleton_score_fusion_policy;
  // how to fuse scores from different gallery frames of the same pose
  uint m_per_gallery_frame_pose_score_fusion_policy;
  double m_threshold;
  ros::Publisher m_publisher, m_rviz_debug_publisher, m_rviz_publisher;


  void
  readGalleryPoses();
  void
  skeletonCallback(
      const opt_msgs::StandardSkeletonTrackArrayConstPtr &standard_data,
      const opt_msgs::SkeletonTrackArrayConstPtr &data);
  void readMatricesForSinglePose(const uint pose_id);
};

} // bpr
} // open_ptrack

#endif

#include <body_pose_recognition/pose_recognition.h>


namespace open_ptrack
{
namespace bpr
{

typedef bpe::SkeletonJoints SkeletonJoints;
typedef bpe::SkeletonLinks SkeletonLinks;

template<typename A, typename B>
std::pair<B,A> flip_pair(const std::pair<A,B> &p)
{
  return std::pair<B,A>(p.second, p.first);
}

template<typename A, typename B>
std::map<B,A> flip_map(const std::map<A,B> &src)
{
  std::map<B,A> dst;
  std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()),
                 flip_pair<A,B>);
  return dst;
}

std::istream& operator>>(std::istream& str, FileRow& data)
{
  data.readNextRow(str);
  return str;
}

PoseRecognition::PoseRecognition():
  m_private_nh("~"),
  m_sk_sub(m_nh, "/tracker/skeleton_tracks", 10),
  m_st_sk_sub(m_nh, "/tracker/standard_skeleton_tracks", 10),
  m_sync(m_st_sk_sub, m_sk_sub, 10)
{
  m_use_left_arm = m_private_nh.param("use_left_arm", true);
  m_use_left_leg = m_private_nh.param("use_left_leg", true);
  m_use_right_arm = m_private_nh.param("use_right_arm", true);
  m_use_right_leg = m_private_nh.param("use_right_leg", true);
  m_threshold = m_private_nh.param("recognition_threshold", 1.0);
  if(not m_use_left_arm and not m_use_left_leg and not m_use_right_arm
     and not m_use_right_leg)
    throw std::runtime_error("No body limbs enabled for recognition");
  m_per_skeleton_score_fusion_policy =
      m_private_nh.param("per_skeleton_score_fusion_policy", 0);
  m_per_gallery_frame_pose_score_fusion_policy =
      m_private_nh.param("per_gallery_frame_pose_score_fusion_policy", 0);
  m_publisher = m_nh.advertise<opt_msgs::PoseRecognitionArray>
      ("/recognizer/poses", 1);
  readGalleryPoses();
  m_rviz_publisher = m_nh.advertise<visualization_msgs::MarkerArray>
      ("/recognizer/markers_debug", 1);
  m_rviz_debug_publisher = m_nh.advertise<visualization_msgs::MarkerArray>
      ("/recognizer/markers", 1);
  m_sync.registerCallback(boost::bind(&PoseRecognition::skeletonCallback,
                                      this, _1, _2));
}

void
PoseRecognition::skeletonCallback
(const opt_msgs::StandardSkeletonTrackArrayConstPtr &standard_data,
 const opt_msgs::SkeletonTrackArrayConstPtr &data)
{

  opt_msgs::PoseRecognitionArray recognition_array_msg;
  visualization_msgs::MarkerArray predicted_pose_marker, marker_array;

  if(data->tracks.size() != standard_data->tracks.size()) return;
  for (size_t skel_id = 0, skel_size = standard_data->tracks.size();
       skel_id != skel_size; ++skel_id)
  {
    const opt_msgs::StandardSkeletonTrack& observed_st_sk =
        standard_data->tracks[skel_id];
    const opt_msgs::SkeletonTrack& sk2 = data->tracks[skel_id];
//    if(sk2.visibility == opt_msgs::StandardSkeletonTrack::OCCLUDED
//       or
//       sk2.visibility == opt_msgs::StandardSkeletonTrack::NOT_VISIBLE)
//      return;

    // sk already in standard pose
    Eigen::Matrix<double, 6, 1> r_arm;
    Eigen::Matrix<double, 6, 1> l_arm;
    Eigen::Matrix<double, 6, 1> r_leg;
    Eigen::Matrix<double, 6, 1> l_leg;
    r_arm << observed_st_sk.joints[SkeletonJoints::RELBOW].x,
        observed_st_sk.joints[SkeletonJoints::RELBOW].y,
        observed_st_sk.joints[SkeletonJoints::RELBOW].z,
        observed_st_sk.joints[SkeletonJoints::RWRIST].x,
        observed_st_sk.joints[SkeletonJoints::RWRIST].y,
        observed_st_sk.joints[SkeletonJoints::RWRIST].z;
    l_arm << observed_st_sk.joints[SkeletonJoints::LELBOW].x,
        observed_st_sk.joints[SkeletonJoints::LELBOW].y,
        observed_st_sk.joints[SkeletonJoints::LELBOW].z,
        observed_st_sk.joints[SkeletonJoints::LWRIST].x,
        observed_st_sk.joints[SkeletonJoints::LWRIST].y,
        observed_st_sk.joints[SkeletonJoints::LWRIST].z;
    r_leg << observed_st_sk.joints[SkeletonJoints::RKNEE].x,
        observed_st_sk.joints[SkeletonJoints::RKNEE].y,
        observed_st_sk.joints[SkeletonJoints::RKNEE].z,
        observed_st_sk.joints[SkeletonJoints::RANKLE].x,
        observed_st_sk.joints[SkeletonJoints::RANKLE].y,
        observed_st_sk.joints[SkeletonJoints::RANKLE].z;
    l_leg << observed_st_sk.joints[SkeletonJoints::LKNEE].x,
        observed_st_sk.joints[SkeletonJoints::LKNEE].y,
        observed_st_sk.joints[SkeletonJoints::LKNEE].z,
        observed_st_sk.joints[SkeletonJoints::LANKLE].x,
        observed_st_sk.joints[SkeletonJoints::LANKLE].y,
        observed_st_sk.joints[SkeletonJoints::LANKLE].z;
    for(uint pose_id = 0, end_id = m_gallery_poses.size(); pose_id != end_id;
        ++pose_id)
    {
      std::array<double, 4> scores ={
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()
      };
      for(uint frame_id = 0, n_frames = m_gallery_poses[pose_id].size();
          frame_id != n_frames; ++frame_id)
      {
        if(m_use_right_arm)
        {
          Eigen::Matrix<double, 6, 1> gallery_frame;
          gallery_frame
              << m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::RELBOW),
              m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::RWRIST);
          scores[0] = (r_arm - gallery_frame).norm();
        }
        if(m_use_left_arm)
        {
          Eigen::Matrix<double, 6, 1> gallery_frame;
          gallery_frame
              << m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::LELBOW),
              m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::LWRIST);
          scores[1] = (l_arm - gallery_frame).norm();
        }
        if(m_use_right_leg)
        {
          Eigen::Matrix<double, 6, 1> gallery_frame;
          gallery_frame
              << m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::RKNEE),
              m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::RANKLE);
          scores[2] = (r_leg - gallery_frame).norm();
        }
        if(m_use_left_leg)
        {
          Eigen::Matrix<double, 6, 1> gallery_frame;
          gallery_frame
              << m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::LKNEE),
              m_gallery_poses[pose_id][frame_id].col(SkeletonJoints::LANKLE);
          scores[3] = (l_leg - gallery_frame).norm();
        }
        switch(m_per_skeleton_score_fusion_policy)
        {
        case 0: // average score policy
        {
          auto acc_lambda = [](const double to_add, double result)
          {
            return std::isnan(to_add)?result:result+to_add;
          };
          m_per_frame_scores[pose_id][frame_id] = std::accumulate(
                scores.begin(), scores.end(), 0.0, acc_lambda)
              / std::count_if(scores.begin(), scores.end(),
                              [](const double v){return not std::isnan(v);});
          break;
        }
        case 1: // worst score policy
        {
          auto max_lambda = [](double a, double b)
          {
            return a < b? true: std::isnan(a);
          };
          m_per_frame_scores[pose_id][frame_id] = *std::max_element(
                scores.begin(), scores.end(), max_lambda);
          break;
        }
        } // switch
      } // for frame_id
      // score fusion per pose
      switch(m_per_gallery_frame_pose_score_fusion_policy)
      {
      case 0: // best match
      {
        m_final_scores[pose_id] =
            *std::min(m_per_frame_scores[pose_id].begin(),
                      m_per_frame_scores[pose_id].end());
        break;
      }
      case 1: // average match
      {
        m_final_scores[pose_id] =
            std::accumulate(m_per_frame_scores[pose_id].begin(),
                            m_per_frame_scores[pose_id].end(), 0.0)
            / m_per_frame_scores[pose_id].size();
        break;
      }
      case 2: // median match
      {
        uint median_id = m_per_frame_scores[pose_id].size() / 2;
        std::nth_element(
              m_per_frame_scores[pose_id].begin(),
              m_per_frame_scores[pose_id].begin() + median_id,
              m_per_frame_scores[pose_id].end());
        m_final_scores[pose_id] = m_per_frame_scores[pose_id][median_id];
        break;
      }
      case 3: //worst match
      {
        m_final_scores[pose_id] =
            *std::max(m_per_frame_scores[pose_id].begin(),
                      m_per_frame_scores[pose_id].end());
      }
      } // switch
    } // for pose_id
    // result
    opt_msgs::PoseRecognition recognition_msg;
    recognition_msg.gallery_poses.resize(m_final_scores.size());
    // sort m_final_scores based on the second field
    std::map<double, size_t> dst = flip_map(m_final_scores);
    // theoretically flip_map should return a multimap,
    // but the probability that two scores are the same is really low
    // When it happens the values are overwritten and I will loose
    // a couple of recognition ids => no problem
    opt_msgs::PosePredictionResult max_pr;
    if ((dst.begin())->first < m_threshold)
    {
      max_pr.pose_id = dst.begin()->second;
      max_pr.pose_name = m_gallery_poses_names[dst.begin()->second];
      max_pr.score = dst.begin()->first;
    }
    else
    {
      max_pr.pose_id = -1;
      max_pr.pose_name = "unknown";
      max_pr.score = -1;
    }
    recognition_msg.best_prediction_result = max_pr;
    for(auto it = dst.begin(), end = dst.end();
        it != end; ++it)
    {
      opt_msgs::PosePredictionResult pr;
      pr.pose_id = it->second;
      pr.pose_name = m_gallery_poses_names[it->second];
      pr.score = it->first;
      recognition_msg.gallery_poses[it->second] = pr;
    }
    recognition_array_msg.poses.push_back(recognition_msg);
    // visualization marker output
    ros::Time time = ros::Time::now();

//    if(sk2.visibility == opt_msgs::StandardSkeletonTrack::OCCLUDED
//       or
//       sk2.visibility == opt_msgs::StandardSkeletonTrack::NOT_VISIBLE)
//      continue;
//    if(Eigen::Vector3d(sk2.joints[SkeletonJoints::CHEST].x,
//                       sk2.joints[SkeletonJoints::CHEST].y,
//                       sk2.joints[SkeletonJoints::CHEST].z).norm() < 0.01f)
//    {
//      continue;
//    }
    visualization_msgs::Marker text_pose_name;
    text_pose_name.header.frame_id = "world";
    text_pose_name.header.stamp = time;
    text_pose_name.ns = "pose_recognition";
    text_pose_name.id = skel_id;
    text_pose_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_pose_name.action = visualization_msgs::Marker::ADD;
    std::stringstream ss;
    ss << recognition_msg.best_prediction_result.pose_name;
    text_pose_name.text = ss.str();
    //    ROS_INFO_STREAM(sk2.joints[SkeletonJoints::CHEST].x
    //        << "," << sk2.joints[SkeletonJoints::CHEST].y
    //        << "," << sk2.joints[SkeletonJoints::CHEST].z);
    text_pose_name.pose.position.x = sk2.joints[SkeletonJoints::CHEST].x;
    text_pose_name.pose.position.y = sk2.joints[SkeletonJoints::CHEST].y;
    text_pose_name.pose.position.z = sk2.joints[SkeletonJoints::CHEST].z + 1.0;
    if (not std::isnormal(fabs(text_pose_name.pose.position.x) +
                      fabs(text_pose_name.pose.position.y) +
                      fabs(text_pose_name.pose.position.z)))
        continue;
    text_pose_name.scale.x = 0.34;
    text_pose_name.scale.y = 0.34;
    text_pose_name.scale.z = 0.34;
    text_pose_name.pose.orientation.x = 0.0;
    text_pose_name.pose.orientation.y = 0.0;
    text_pose_name.pose.orientation.z = 0.0;
    text_pose_name.pose.orientation.w = 1.0;
    text_pose_name.color = sk2.color;

    predicted_pose_marker.markers.push_back(text_pose_name);
    for(size_t k = 0, size_k = recognition_msg.gallery_poses.size();
        k != size_k; ++k)
    {
      const opt_msgs::PosePredictionResult& ppr =
          recognition_msg.gallery_poses[k];
      visualization_msgs::Marker text_pose_id;
      visualization_msgs::Marker text_pose_score;
      text_pose_id.header.frame_id = "world";
      text_pose_id.header.stamp = time;
      text_pose_score.header = text_pose_id.header;
      text_pose_score.ns = "recognition_score";
      text_pose_id.ns = "recognition_id";
      text_pose_score.id = text_pose_id.id =
          k + skel_id * size_k;
      text_pose_score.type = text_pose_id.type =
          visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_pose_score.action = text_pose_id.action =
          visualization_msgs::Marker::ADD;
      std::stringstream ss;
      ss << ppr.pose_name;
      text_pose_id.text = ss.str();
      std::stringstream ss2;
      ss2 << ppr.score;
      text_pose_score.text = ss2.str();
      text_pose_score.pose.position.x = sk2.joints[SkeletonJoints::CHEST].x;
      text_pose_score.pose.position.y = sk2.joints[SkeletonJoints::CHEST].y;
      text_pose_score.pose.position.z =
          sk2.joints[SkeletonJoints::CHEST].z + 1.0 + (k + 1) * 0.2;
      //      text_pose_score.pose.position.x = 0.0;
      //      text_pose_score.pose.position.y = 0.0;
      //      text_pose_score.pose.position.z = (k + 1) * 1.0;
      text_pose_score.pose.orientation.x = 0.0;
      text_pose_score.pose.orientation.y = 0.0;
      text_pose_score.pose.orientation.z = 0.0;
      text_pose_score.pose.orientation.w = 1.0;
      text_pose_id.pose = text_pose_score.pose;
      text_pose_id.pose.position.x += 2.0;
      text_pose_score.scale.x = 0.34;
      text_pose_score.scale.y = 0.34;
      text_pose_score.scale.z = 0.34;
      text_pose_score.color = sk2.color;
      if (ppr.score < m_threshold)
      {
        text_pose_score.color.r = 0.0;
        text_pose_score.color.g = 1.0;
        text_pose_score.color.b = 0.0;
        text_pose_score.color.a = 1.0;
      }
      text_pose_score.lifetime = ros::Duration(0.2);
      text_pose_id.color = text_pose_score.color;
      text_pose_id.scale = text_pose_score.scale;
      text_pose_id.lifetime = text_pose_score.lifetime;
      marker_array.markers.push_back(text_pose_id);
      marker_array.markers.push_back(text_pose_score);
    }
    m_rviz_publisher.publish(marker_array);
    m_rviz_debug_publisher.publish(predicted_pose_marker);
  } // standard tracks (skel_id)
  // publish the result
  recognition_array_msg.header.frame_id = data->header.frame_id;
  recognition_array_msg.header.stamp = data->header.stamp;
  m_publisher.publish(recognition_array_msg);
  // Visualization message
}

void
PoseRecognition::readGalleryPoses()
{
  std::ifstream poses_file((ros::package::getPath("gallery_poses")
                            + "/data/poses.csv").c_str());
  if (not poses_file.good())
    throw std::runtime_error("poses.csv in gallery_poses/data "
                             "not found");

  FileRow row(',');
  poses_file >> row;
//    m_gallery_poses_names.resize(std::atoi(row[0].c_str()));
//    m_gallery_poses.resize(m_gallery_poses_names.size());
//    m_per_frame_scores.resize(m_gallery_poses.size());
//    m_final_scores.resize(m_gallery_poses.size());
  //  std::vector<std::string> dirs_to_read(m_gallery_poses_names.size());
  //  std::vector<std::string> frames_to_read(m_gallery_poses_names.size());
  while(poses_file >> row)
  {
    const uint pose_id = std::atoi(row[0].c_str());
    const uint n_frames = std::atoi(row[1].c_str());
    const std::string& pose_name = row[2];

    std::cout << "Reading: " << pose_id << " named \""
              << pose_name << "\"" << std::endl;
    // check if exists and if there are frames
    std::stringstream ss;
    ss << ros::package::getPath("gallery_poses") << "/data/" << pose_id;
    boost::filesystem::path p{ss.str()};
    if(not boost::filesystem::is_directory(p)) continue;
    size_t are_there_any_frames = std::count_if(
          boost::filesystem::directory_iterator(p),
          boost::filesystem::directory_iterator(),
          static_cast<bool(*)(const boost::filesystem::path&)>
          (boost::filesystem::is_regular_file));
    if(are_there_any_frames == 0) continue;
    m_gallery_poses[pose_id] =
        std::vector<SkeletonMatrix>(are_there_any_frames / 2);
    m_gallery_poses_names[pose_id] = pose_name;
    m_per_frame_scores[pose_id] = std::vector<double>(are_there_any_frames / 2);
    readMatricesForSinglePose(pose_id);

  }
}

void
PoseRecognition::readMatricesForSinglePose(const uint pose_id)
{
  std::stringstream ss;
  ss << ros::package::getPath("gallery_poses") + "/data/" << pose_id;
  uint frame_id = 0;
  boost::filesystem::path path(ss.str());
  for (auto i = boost::filesystem::directory_iterator(path);
       i != boost::filesystem::directory_iterator(); i++)
  {
    if (not boost::filesystem::is_directory(i->path())
        and
        i->path().filename().string().find(".txt") !=
        std::string::npos) //we eliminate directories and not matrices
    {
      std::stringstream ss2;
      ss2 << ros::package::getPath("gallery_poses") + "/data/" << pose_id
          << "/" << i->path().filename().string();
      std::ifstream matrix_file(ss2.str());
      if (not matrix_file.good())
      {
        throw std::runtime_error(ss2.str()
                                 + " not found! Check your gallery_poses");
      }
      FileRow row(' ');
      for (uint r = 0; r < 3; ++r)
      {
        matrix_file >> row;
        for(uint c = 0; c < SkeletonJoints::SIZE; ++c)
        {
          m_gallery_poses[pose_id][frame_id](r,c) = std::stod(row[c]);
        }
      }
      frame_id++;
    }
  }
}

} // bpr
} // open_ptrack


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_recognition_test");

  open_ptrack::bpr::PoseRecognition pr;
  ros::spin();

  ros::shutdown();
  return 0;
}

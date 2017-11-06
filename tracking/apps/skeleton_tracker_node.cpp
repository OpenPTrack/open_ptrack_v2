/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011-2012, Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marco Carraro [carraromarco89@gmail.com]
 *
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <list>
#include <sstream>
#include <fstream>
#include <string.h>
#include <mutex>

#include <open_ptrack/opt_utils/conversions.h>
#include <open_ptrack/detection/skeleton_detection.h>
#include <open_ptrack/detection/detection_source.h>
#include <open_ptrack/tracking/skeleton_tracker.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/SkeletonTrackArray.h>
#include <opt_msgs/StandardSkeletonTrackArray.h>
#include <opt_msgs/IDArray.h>
#include <rtpose_wrapper/SkeletonMsg.h>
#include <rtpose_wrapper/SkeletonArrayMsg.h>
#include <body_pose_recognition/standardpose.h>
//#include <open_ptrack/opt_utils/ImageConverter.h>

// Dynamic reconfigure:
#include <dynamic_reconfigure/server.h>
#include <tracking/SkeletonTrackerConfig.h>

typedef tracking::SkeletonTrackerConfig Config;
typedef dynamic_reconfigure::Server<Config> ReconfigureServer;

// Global variables:
std::map<std::string, open_ptrack::detection::DetectionSource*>
detection_sources_map;
tf::TransformListener* tf_listener;
std::string world_frame_id;
bool output_history_pointcloud;
int output_history_size;
int detection_history_size;
bool output_markers;
bool output_image_rgb;
bool output_tracking_results;
// Enables/disables the publishing of detection positions to be visualized in
// RViz
bool output_detection_results;
bool vertical;
ros::Publisher results_pub;
ros::Publisher marker_pub;
ros::Publisher standard_skel_pub;
ros::Publisher standard_skel_markers_pub;
ros::Publisher pointcloud_pub;
ros::Publisher skeleton_detection_centroid_marker_pub;
ros::Publisher skeleton_detection_markers_pub;
ros::Publisher detection_trajectory_pub;
ros::Publisher alive_ids_pub;
size_t starting_index;
size_t detection_insert_index;
tf::Transform camera_frame_to_world_transform;
tf::Transform world_to_camera_frame_transform;
bool extrinsic_calibration;
double period;
open_ptrack::tracking::SkeletonTracker* tracker;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr history_pointcloud
(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr detection_history_pointcloud
(new pcl::PointCloud<pcl::PointXYZRGB>);
bool swissranger;
double min_confidence;
double min_confidence_sr;
double min_confidence_detections;
double min_confidence_detections_sr;
// vector containing colors to use to identify cameras in the network
std::vector<cv::Vec3f> camera_colors;
// map between camera frame_id and color
std::map<std::string, int> color_map;
// Chi square distribution
std::map<double, double> chi_map;
bool velocity_in_motion_term;
double acceleration_variance;
bool remove_head_in_rviz;
double position_variance_weight;
double voxel_size;
double gate_distance;
bool calibration_refinement;
std::map<std::string, Eigen::Matrix4d> registration_matrices;
double max_detection_delay;
ros::Time latest_time;
int delete_old_markers_factor_ = 10;
uint num_frames_ = 0;
std::map<std::string, ros::Time> last_received_detection_;
ros::Duration max_time_between_detections_;
double _min_confidence_per_joint;

visualization_msgs::MarkerArray::Ptr marker_array_msg_ = nullptr;

std::map<std::string, std::pair<double, int> > number_messages_delay_map_;

using namespace open_ptrack::bpe;

/**
 * \brief Create marker to be visualized in RViz
 *
 * \param[in] id The marker ID.
 * \param[in] frame_id The marker reference frame.
 * \param[in] position The marker position.
 * \param[in] color The marker color.
 */
visualization_msgs::Marker
createMarker (int id, const std::string& frame_id,
              const ros::Time& stamp,
              const Eigen::Vector3d& position, const cv::Vec3f& color)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = world_frame_id;
  marker.header.stamp = stamp;
  marker.ns = frame_id;
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position(0);
  marker.pose.position.y = position(1);
  marker.pose.position.z = position(2);
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.2);

  return marker;
}

void
createVisMarker
(visualization_msgs::MarkerArray& skeleton_detection_msg,
 uint skel_index,
 const open_ptrack::detection::SkeletonDetection& skel_det,
 const cv::Vec3f& color)
{
  ros::Time time = ros::Time::now();
  // Joint Markers
  for(int i = 0, end = SkeletonJoints::SIZE; i != end; ++i)
  {
    if (remove_head_in_rviz)
      if (i == SkeletonJoints::HEAD) continue;
    const rtpose_wrapper::Joint3DMsg& j = skel_det.getSkeletonMsg().joints[i];
    if (not std::isfinite(j.x + j.y + j.z)) continue;
    visualization_msgs::Marker joint_marker;
    joint_marker.header.frame_id = "world";
    joint_marker.header.stamp = time;
    joint_marker.ns = "joints";
    joint_marker.id = i + skel_index * SkeletonJoints::SIZE;
    joint_marker.type = visualization_msgs::Marker::SPHERE;
    joint_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = j.x; // just for better visualization
    p.y = j.y; // just for better visualization
    p.z = j.z; // just for better visualization
    joint_marker.pose.position = p;
    joint_marker.pose.orientation.x = 0.0;
    joint_marker.pose.orientation.y = 0.0;
    joint_marker.pose.orientation.z = 0.0;
    joint_marker.pose.orientation.w = 1.0;
    joint_marker.scale.x = 0.06;
    joint_marker.scale.y = 0.06;
    joint_marker.scale.z = 0.06;
    joint_marker.color.r = color(0);
    joint_marker.color.g = color(1);
    joint_marker.color.b = color(2);
    joint_marker.color.a = 1.0;

    joint_marker.lifetime = ros::Duration(0.2);

    skeleton_detection_msg.markers.push_back(joint_marker);
  }
  // Link markers
  for(auto it = SkeletonLinks::LINKS.begin(),
      end = SkeletonLinks::LINKS.end();
      it != end; ++it)
  {
    if (remove_head_in_rviz)
      if (it->first == SkeletonJoints::HEAD
          or it->second == SkeletonJoints::HEAD)
        continue;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    const rtpose_wrapper::Joint3DMsg& j1 =
        skel_det.getSkeletonMsg().joints[it->first];
    const rtpose_wrapper::Joint3DMsg& j2 =
        skel_det.getSkeletonMsg().joints[it->second];
    if (not std::isfinite(j1.x + j1.y + j1.z)
        or not std::isfinite(j2.x + j2.y + j2.z)) continue;
    p1.x = j1.x;
    p1.y = j1.y;
    p1.z = j1.z;
    p2.x = j2.x;
    p2.y = j2.y;
    p2.z = j2.z;
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "world";
    line_marker.header.stamp = time;
    line_marker.ns = "links";
    line_marker.id = it - SkeletonLinks::LINKS.begin()
        + skel_index * SkeletonLinks::LINKS.size();
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.x = 0.0;
    line_marker.pose.orientation.y = 0.0;
    line_marker.pose.orientation.z = 0.0;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;
    line_marker.scale.y = 0.1;
    line_marker.scale.z = 0.1;
    line_marker.color.r = color(0);
    line_marker.color.g = color(1);
    line_marker.color.b = color(2);
    line_marker.color.a = 1.0;
    line_marker.lifetime = ros::Duration(0.2);
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    line_marker.colors.push_back(line_marker.color);
    line_marker.colors.push_back(line_marker.color);
    skeleton_detection_msg.markers.push_back(line_marker);
  }
}

void
createVisMarker
(visualization_msgs::MarkerArray& vis_markers_array,
 uint skel_index, double orientation,
 const Eigen::Vector2d& orientation_vector,
 const Eigen::Matrix<double, 3, SkeletonJoints::SIZE>& normalized_skeleton,
 const ros::Time& time = ros::Time::now())
{
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "world";
  text_marker.header.stamp = time;
  text_marker.ns = "orientation_angle";
  text_marker.id = skel_index;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  std::stringstream ss;
  ss << -orientation;
  text_marker.text = ss.str();
  text_marker.pose.position.x = 0.0;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 0.0;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.x = 0.34;
  text_marker.scale.y = 0.34;
  text_marker.scale.z = 0.34;
  text_marker.color.r = 1.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  text_marker.lifetime = ros::Duration(0.2);

  vis_markers_array.markers.push_back(text_marker);

  // if(orientation_vector.hasNaN()) continue;
  // Link markers
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "world";
  line_marker.header.stamp = time;
  line_marker.ns = "orientation_vectors";
  line_marker.id = skel_index;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.x = 0.0;
  line_marker.pose.orientation.y = 0.0;
  line_marker.pose.orientation.z = 0.0;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.1;
  line_marker.scale.y = 0.1;
  line_marker.scale.z = 0.1;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  //        line_marker.color.r = color_(2);
  //        line_marker.color.g = color_(1);
  //        line_marker.color.b = color_(0);
  line_marker.color.a = 1.0;

  line_marker.lifetime = ros::Duration(0.2);
  geometry_msgs::Point p;
  p.x = p.y = p.z = 0.0;
  line_marker.points.push_back(p);
  p.x = 10 * orientation_vector[0];
  p.y = 10 * orientation_vector[1];
  line_marker.points.push_back(p);
  vis_markers_array.markers.push_back(line_marker);

  // Normalized skeleton
  if(normalized_skeleton.hasNaN()) return;
  const Eigen::Matrix<double, 3, SkeletonJoints::SIZE>& joints =
      normalized_skeleton;

  double scale_factor = 0.1; // just forvisualization

  // Joint Markers
  for(int i = 0, end = SkeletonJoints::SIZE; i != end; ++i)
  {
    if (remove_head_in_rviz)
      if (i == SkeletonJoints::HEAD) continue;
    visualization_msgs::Marker joint_marker;
    joint_marker.header.frame_id = "world";
    joint_marker.header.stamp = time;
    joint_marker.ns = "joints_norm";
    joint_marker.id = i; //for visualizing both detection and tracks
    joint_marker.type = visualization_msgs::Marker::SPHERE;
    joint_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point p;
    p.x = scale_factor * joints(0,i); // just for better visualization
    p.y = scale_factor * joints(1,i); // just for better visualization
    p.z = scale_factor * joints(2,i); // just for better visualization
    joint_marker.pose.position = p;
    joint_marker.pose.orientation.x = 0.0;
    joint_marker.pose.orientation.y = 0.0;
    joint_marker.pose.orientation.z = 0.0;
    joint_marker.pose.orientation.w = 1.0;
    joint_marker.scale.x = 0.06;
    joint_marker.scale.y = 0.06;
    joint_marker.scale.z = 0.06;
    joint_marker.color.r = 0.0;
    joint_marker.color.g = 0.0;
    joint_marker.color.b = 0.0;
    joint_marker.color.a = 1.0;

    joint_marker.lifetime = ros::Duration(0.2);

    vis_markers_array.markers.push_back(joint_marker);
  }
  // Link markers
  for(auto it = SkeletonLinks::LINKS.begin(),
      end = SkeletonLinks::LINKS.end();
      it != end; ++it)
  {
    if (remove_head_in_rviz)
      if (it->first == SkeletonJoints::HEAD
          or it->second == SkeletonJoints::HEAD)
        continue;
    size_t id = it - SkeletonLinks::LINKS.begin();
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    const Eigen::Vector3d eigen_p1 = joints.col(it->first);
    const Eigen::Vector3d eigen_p2 = joints.col(it->second);
    p1.x = scale_factor * eigen_p1(0);
    p1.y = scale_factor * eigen_p1(1);
    p1.z = scale_factor * eigen_p1(2);
    p2.x = scale_factor * eigen_p2(0);
    p2.y = scale_factor * eigen_p2(1);
    p2.z = scale_factor * eigen_p2(2);
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "world";
    line_marker.header.stamp = time;
    line_marker.ns = "links_norm";
    line_marker.id = id;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.pose.orientation.x = 0.0;
    line_marker.pose.orientation.y = 0.0;
    line_marker.pose.orientation.z = 0.0;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;
    line_marker.scale.y = 0.1;
    line_marker.scale.z = 0.1;
    line_marker.color.r = 1.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 1.0;
    line_marker.color.a = 1.0;

    std_msgs::ColorRGBA color_random;
    color_random.a = color_random.r = color_random.g = color_random.b = 1.0;

    line_marker.lifetime = ros::Duration(0.2);
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    if(it->first == SkeletonJoints::RELBOW
       or it->second == SkeletonJoints::RELBOW)
    {
      color_random.r = 1.0; color_random.b = color_random.g = 0.0;
    }
    line_marker.colors.push_back(color_random);
    line_marker.colors.push_back(color_random);
    vis_markers_array.markers.push_back(line_marker);
    //      }
  }
}

void
plotCameraLegend (const std::map<std::string, int>& curr_color_map)
{
  // Compose camera legend:
  cv::Mat legend_image = cv::Mat::zeros(500, 500, CV_8UC3);
  for(std::map<std::string, int>::const_iterator colormap_iterator =
      curr_color_map.begin(); colormap_iterator != curr_color_map.end();
      colormap_iterator++)
  {
    int color_index = colormap_iterator->second;
    cv::Vec3f color = camera_colors[color_index];
    int y_coord = color_index * legend_image.rows /
        (curr_color_map.size()+1) +
        0.5 * legend_image.rows / (curr_color_map.size()+1);
    cv::line(legend_image, cv::Point(0,y_coord), cv::Point(100,y_coord),
             cv::Scalar(255*color(2), 255*color(1), 255*color(0)), 8);
    cv::putText(legend_image, colormap_iterator->first, cv::Point(110,y_coord),
                1, 1, cv::Scalar(255, 255, 255), 1);
  }

  // Display the cv image
  cv::imshow("Camera legend", legend_image);
  cv::waitKey(1);
}

Eigen::Matrix4d
readMatrixFromFile (const std::string& filename)
{
  Eigen::Matrix4d matrix;
  std::string line;
  std::ifstream myfile (filename.c_str());
  if (myfile.is_open())
  {
    int k = 0;
    std::string number;
    while (myfile >> number)
    {
      matrix(int(k/4), int(k%4)) = std::atof(number.c_str());
      k++;
    }
    myfile.close();
  }

  std::cout << matrix << std::endl;

  return matrix;
}

bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}


/**
 * \brief Read the DetectionArray message and use the detections
 * for creating/updating/deleting tracks
 *
 * \param[in] msg the DetectionArray message.
 */
void
detection_cb(const rtpose_wrapper::SkeletonArrayMsg::ConstPtr& msg)
{
  // Read message header information:
  std::string frame_id = msg->header.frame_id;
  ros::Time frame_time = msg->header.stamp;

  std::string frame_id_tmp = frame_id;
  int pos = frame_id_tmp.find("_rgb_optical_frame");
  if (pos != std::string::npos)
    frame_id_tmp.replace(pos, std::string("_rgb_optical_frame").size(), "");
  pos = frame_id_tmp.find("_ir_optical_frame");
  if (pos != std::string::npos)
    frame_id_tmp.replace(pos, std::string("_ir_optical_frame").size(), "");
  pos = frame_id_tmp.find("_depth_optical_frame");
  if (pos != std::string::npos)
    frame_id_tmp.replace(pos, std::string("_depth_optical_frame").size(), "");
  last_received_detection_[frame_id_tmp] = frame_time;

  // Compute delay of detection message, if any:
  double time_delay = 0.0;
  if (frame_time > latest_time)
  {
    latest_time = frame_time;
    time_delay = 0.0;
  }
  else
  {
    time_delay = (latest_time - frame_time).toSec();
  }

  tf::StampedTransform transform;
  tf::StampedTransform inverse_transform;
  //	cv_bridge::CvImage::Ptr cvPtr;

  try
  {
    // Read transforms between camera frame and world frame:
    if (!extrinsic_calibration)
    {
      static tf::TransformBroadcaster world_to_camera_tf_publisher;
      world_to_camera_tf_publisher.sendTransform
          (tf::StampedTransform(world_to_camera_frame_transform,
                                ros::Time::now(), frame_id_tmp + "_ir_optical_frame", world_frame_id));
    }

    //Calculate direct and inverse transforms between camera and world frame:
    tf_listener->lookupTransform(world_frame_id, frame_id, ros::Time(0),
                                 transform);
    tf_listener->lookupTransform(frame_id, world_frame_id, ros::Time(0),
                                 inverse_transform);

    // Read camera intrinsic parameters:
    Eigen::Matrix3d intrinsic_matrix;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        intrinsic_matrix(i, j) = msg->intrinsic_matrix[i * 3 + j];

    // Add a new DetectionSource or update an existing one:
    if(detection_sources_map.find(frame_id) == detection_sources_map.end())
    {
      detection_sources_map[frame_id] =
          new open_ptrack::detection::DetectionSource(cv::Mat(0, 0, CV_8UC3),
                                                      transform,
                                                      inverse_transform,
                                                      intrinsic_matrix,
                                                      frame_time, frame_id);
    }
    else
    {
      detection_sources_map[frame_id]->update(cv::Mat(0, 0, CV_8UC3),
                                              transform, inverse_transform,
                                              intrinsic_matrix, frame_time,
                                              frame_id);
      double d = detection_sources_map[frame_id]->getDuration().toSec()
          / period;
      int lostFrames = int(round(d)) - 1;
    }
    open_ptrack::detection::DetectionSource* source =
        detection_sources_map[frame_id];

    // Create a SkeletonDetection object
    // for every skeleton in the detection message:
    std::vector<open_ptrack::detection::SkeletonDetection> detections_vector;
    for(std::vector<rtpose_wrapper::SkeletonMsg>::const_iterator
        it = msg->skeletons.begin(), end = msg->skeletons.end();
        it != end; it++)
    {
      detections_vector.push_back(
            open_ptrack::detection::SkeletonDetection(*it, source));
      open_ptrack::detection::SkeletonDetection& lst = detections_vector.back();
      for(uint i = 0, size = lst.getSkeletonMsg().joints.size(); i < size; ++i)
      {
        rtpose_wrapper::Joint3DMsg& j = lst.getSkeletonMsg().joints[i];
        if (j.confidence < _min_confidence_per_joint)
        {
          j.x = j.y = j.z = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }

    // Detection correction by means of calibration refinement:
    if (calibration_refinement)
    {
      ROS_INFO_STREAM("Ref correction");
      if (strcmp(frame_id.substr(0,1).c_str(), "/") == 0)
      {
        frame_id = frame_id.substr(1, frame_id.size() - 1);
      }

      Eigen::Matrix4d registration_matrix;
      std::map<std::string, Eigen::Matrix4d>::iterator
          registration_matrices_iterator = registration_matrices.find(frame_id);
      if (registration_matrices_iterator != registration_matrices.end())
      { // camera already present
        registration_matrix = registration_matrices_iterator->second;
      }
      else
      { // camera not present
        std::cout << "SKeletonTracker: Reading refinement matrix of "
                  << frame_id_tmp + "_ir_optical_frame" << " from file." << std::endl;
        std::string refinement_filename =
            ros::package::getPath("opt_calibration") +
            "/conf/registration_" + frame_id_tmp + "_ir_optical_frame.txt";
        std::ifstream f(refinement_filename.c_str());
        if (f.good()) // if the file exists
        {
          f.close();
          registration_matrix = readMatrixFromFile (refinement_filename);
          registration_matrices.insert(std::pair<std::string, Eigen::Matrix4d>
                                       (frame_id, registration_matrix));
        }
        else  // if the file does not exist
        {
          // insert the identity matrix
          std::cout << "Refinement file not found! "
                       "Not doing refinement for this sensor." << std::endl;
          registration_matrices.insert(std::pair<std::string, Eigen::Matrix4d>
                                       (frame_id, Eigen::Matrix4d::Identity()));
        }
      }

      if(detections_vector.size() > 0)
      {
        // Apply detection refinement:
        for(unsigned int i = 0; i < detections_vector.size(); i++)
        {
          Eigen::Vector3d old_centroid =
              detections_vector[i].getWorldCentroid();

          Eigen::Vector4d old_centroid_homogeneous
              (old_centroid(0), old_centroid(1), old_centroid(2), 1.0);
          Eigen::Vector4d refined_centroid = registration_matrix
              * old_centroid_homogeneous;
          detections_vector[i].setWorldCentroid(
                Eigen::Vector3d(refined_centroid(0), refined_centroid(1),
                                refined_centroid(2)));
          detections_vector[i].refineSkeletonJoints(registration_matrix);
        }
      }
    }
    // If at least one detection has been received:
    if((detections_vector.size() > 0) && (time_delay < max_detection_delay))
    {
      ros::Time begin(ros::Time::now());
      // Perform detection-track association:
      tracker->newFrame(detections_vector);
      tracker->updateTracks();
      ros::Duration end = ros::Time::now() - begin;

      // Create a TrackingResult message with the output of the tracking process
      if(output_tracking_results)
      {
        opt_msgs::SkeletonTrackArray::Ptr
            tracking_results_msg(new opt_msgs::SkeletonTrackArray);
        opt_msgs::StandardSkeletonTrackArray::Ptr
            standard_tracking_results_msg
            (new opt_msgs::StandardSkeletonTrackArray);
        ros::Time frame_time(ros::Time::now());
        tracking_results_msg->header.stamp = frame_time;
        tracking_results_msg->header.frame_id = world_frame_id;
        tracker->toMsg(tracking_results_msg);
        // Publish tracking message:
        results_pub.publish(tracking_results_msg);
        // Publish standard skeleton tracks (if needed)
        visualization_msgs::MarkerArray vis_markers_array,
            skeleton_detection_msg;
        standard_tracking_results_msg->tracks.resize
            ( tracking_results_msg->tracks.size());
        standard_tracking_results_msg->header.frame_id =
            tracking_results_msg->header.frame_id;
        standard_tracking_results_msg->header.stamp =
            tracking_results_msg->header.stamp;
        if(standard_skel_pub.getNumSubscribers() > 0
           or standard_skel_markers_pub.getNumSubscribers() > 0)
        {
          for(uint i = 0; i < tracking_results_msg->tracks.size(); ++i)
          {
            const open_ptrack::bpr::StandardPose sp
                (tracking_results_msg->tracks[i]);
            opt_msgs::StandardSkeletonTrack new_t = sp.getStandardTrackMsg();
            const double& orientation = sp.getOrientationAngle();
            const Eigen::Vector3d& orientation_vector =
                sp.getOrientationVector();
            createVisMarker(vis_markers_array, i, orientation,
                            orientation_vector.head<2>(),
                            sp.getSkeleton().getJoints(),
                            frame_time);
            standard_tracking_results_msg->tracks[i] = new_t;
          }
        }
        standard_skel_pub.publish(standard_tracking_results_msg);
        standard_skel_markers_pub.publish(vis_markers_array);
      }

      // Publish IDs of active tracks:
      opt_msgs::IDArray::Ptr alive_ids_msg(new opt_msgs::IDArray);
      alive_ids_msg->header.stamp = ros::Time::now();
      alive_ids_msg->header.frame_id = world_frame_id;
      tracker->getAliveIDs (alive_ids_msg);
      alive_ids_pub.publish (alive_ids_msg);

      // Show the pose of each tracked object with a 3D marker (to be visualized with ROS RViz)
      if(output_markers)
      {
        marker_array_msg_.reset(new visualization_msgs::MarkerArray);
        tracker->toMarkerArray(marker_array_msg_, remove_head_in_rviz);
        marker_pub.publish(marker_array_msg_);
      }

      // Show the history of the movements in 3D (3D trajectory) of each tracked object as a PointCloud (which can be visualized in RViz)
      if(output_history_pointcloud)
      {
        history_pointcloud->header.stamp = frame_time.toNSec() / 1e3;  // Convert from ns to us
        history_pointcloud->header.frame_id = world_frame_id;
        starting_index = tracker->appendToPointCloud(history_pointcloud, starting_index,
                                                     output_history_size);
        pointcloud_pub.publish(history_pointcloud);
      }

      // Create message for showing detection positions in RViz:
      if (output_detection_results)
      {
        visualization_msgs::MarkerArray::Ptr marker_msg(new visualization_msgs::MarkerArray);
        detection_history_pointcloud->header.stamp = frame_time.toNSec() / 1e3;  // Convert from ns to us
        detection_history_pointcloud->header.frame_id = world_frame_id;
        std::string frame_id = detections_vector[0].getSource()->getFrameId();

        // Define color:
        int color_index;
        std::map<std::string, int>::iterator colormap_iterator = color_map.find(frame_id);
        if (colormap_iterator != color_map.end())
        { // camera already present
          color_index = colormap_iterator->second;
        }
        else
        { // camera not present
          color_index = color_map.size();
          color_map.insert(std::pair<std::string, int> (frame_id, color_index));

          // Plot legend with camera names and colors:
          plotCameraLegend (color_map);
        }
        for (unsigned int i = 0; i < detections_vector.size(); i++)
        {
          // Create marker and add it to message:
          Eigen::Vector3d centroid = detections_vector[i].getWorldCentroid();
          visualization_msgs::Marker marker =
              createMarker (i, frame_id, frame_time, centroid,
                            camera_colors[color_index]);
          marker_msg->markers.push_back(marker);

          // Point cloud:
          pcl::PointXYZRGB point;
          point.x = marker.pose.position.x;
          point.y = marker.pose.position.y;
          point.z = marker.pose.position.z;
          point.r = marker.color.r * 255.0f;
          point.g = marker.color.g * 255.0f;
          point.b = marker.color.b * 255.0f;
          detection_insert_index = (detection_insert_index + 1) % detection_history_size;
          detection_history_pointcloud->points[detection_insert_index] = point;

          if (skeleton_detection_markers_pub.getNumSubscribers() > 0)
          {
            visualization_msgs::MarkerArray skeleton_detection_msg;

            for(uint i = 0; i < detections_vector.size(); ++i)
            {
              createVisMarker(skeleton_detection_msg,i,detections_vector[i],
                              camera_colors[color_index]);
            }
            skeleton_detection_markers_pub.publish(skeleton_detection_msg);
          }
        }
        skeleton_detection_centroid_marker_pub.publish(marker_msg); // publish marker message
        detection_trajectory_pub.publish(detection_history_pointcloud); // publish trajectory message
      }
    }
    else // if no detections have been received or detection_delay is above max_detection_delay
    {
      if(output_tracking_results)
      { // Publish an empty tracking message
        opt_msgs::SkeletonTrackArray::Ptr tracking_results_msg(new opt_msgs::SkeletonTrackArray);
        tracking_results_msg->header.stamp = frame_time;
        tracking_results_msg->header.frame_id = world_frame_id;
        results_pub.publish(tracking_results_msg);
      }
      if((detections_vector.size() > 0) && (time_delay >= max_detection_delay))
      {
        if (number_messages_delay_map_.find(msg->header.frame_id) == number_messages_delay_map_.end())
          number_messages_delay_map_[msg->header.frame_id] = std::pair<double, int>(0.0, 0);

        number_messages_delay_map_[msg->header.frame_id].first += time_delay;
        number_messages_delay_map_[msg->header.frame_id].second++;

        if (number_messages_delay_map_[msg->header.frame_id].second == 100)
        {
          double avg = number_messages_delay_map_[msg->header.frame_id].first / number_messages_delay_map_[msg->header.frame_id].second;
          ROS_WARN_STREAM("[" << msg->header.frame_id << "] received 100 detections with average delay " << avg << " > " << max_detection_delay);
          number_messages_delay_map_[msg->header.frame_id] = std::pair<double, int>(0.0, 0);
        }
      }
    }
  }
  //  catch(cv_bridge::Exception& ex)
  //  {
  //    ROS_ERROR("cv_bridge exception: %s", ex.what());
  //  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("transform exception: %s", ex.what());
  }
}

void
generateColors(int colors_number, std::vector<cv::Vec3f>& colors)
{
  for (unsigned int i = 0; i < colors_number; i++)
  {
    colors.push_back(cv::Vec3f(
                       float(rand() % 256) / 255,
                       float(rand() % 256) / 255,
                       float(rand() % 256) / 255));
  }
}

void
fillChiMap(std::map<double, double>& chi_map, bool velocity_in_motion_term)
{
  if (velocity_in_motion_term)		// chi square values with state dimension = 4
  {
    chi_map[0.5] = 3.357;
    chi_map[0.75] = 5.385;
    chi_map[0.8] = 5.989;
    chi_map[0.9] = 7.779;
    chi_map[0.95] = 9.488;
    chi_map[0.98] = 11.668;
    chi_map[0.99] = 13.277;
    chi_map[0.995] = 14.860;
    chi_map[0.998] = 16.924;
    chi_map[0.999] = 18.467;
  }
  else							              // chi square values with state dimension = 2
  {
    chi_map[0.5] = 1.386;
    chi_map[0.75] = 2.773;
    chi_map[0.8] = 3.219;
    chi_map[0.9] = 4.605;
    chi_map[0.95] = 5.991;
    chi_map[0.98] = 7.824;
    chi_map[0.99] = 9.210;
    chi_map[0.995] = 10.597;
    chi_map[0.998] = 12.429;
    chi_map[0.999] = 13.816;
  }
}

void
fillChiMap3D(std::map<double, double>& chi_map, bool velocity_in_motion_term)
{
  if (velocity_in_motion_term)		// chi square values with state dimension = 6
  {
    chi_map[0.5] = 5.348157744978145;
    chi_map[0.75] = 7.840834359138661;
    chi_map[0.8] = 8.558055165624488;
    chi_map[0.9] = 10.64489261044355;
    chi_map[0.95] = 12.591393389500972;
    chi_map[0.98] = 15.032781872114004;
    chi_map[0.99] = 16.811720740614913;
    chi_map[0.995] = 18.547667872189702;
    chi_map[0.998] = 20.791761918616732;
    chi_map[0.999] = 22.459125687950245;
  }
  else							              // chi square values with state dimension = 3
  {
    chi_map[0.5] = 2.3659773627544762;
    chi_map[0.75] = 4.108376448120975;
    chi_map[0.8] = 4.641569733033475;
    chi_map[0.9] = 6.251462635371571;
    chi_map[0.95] = 7.814674086652431;
    chi_map[0.98] = 9.837546768092276;
    chi_map[0.99] = 11.345192964279828;
    chi_map[0.995] = 12.838203077572782;
    chi_map[0.998] = 14.796089301316622;
    chi_map[0.999] = 16.26733195007081;
  }
}

void
configCb(Config &config, uint32_t level)
{
  _min_confidence_per_joint = config.min_confidence_per_skeleton_joint;
  tracker->setMinConfidenceForTrackInitialization
      (config.min_confidence_initialization);
  max_detection_delay = config.max_detection_delay;
  calibration_refinement = config.calibration_refinement;
  tracker->setSecBeforeOld (config.sec_before_old);
  tracker->setSecBeforeFake (config.sec_before_fake);
  tracker->setSecRemainNew (config.sec_remain_new);
  tracker->setDetectionsToValidate (config.detections_to_validate);
  tracker->setDetectorLikelihood (config.detector_likelihood);
  tracker->setLikelihoodWeights
      (config.detector_weight*chi_map[0.999]/18.467, config.motion_weight);

  if (config.acceleration_variance != acceleration_variance)
  {
    tracker->setAccelerationVariance (config.acceleration_variance);
  }

  if (config.position_variance_weight != position_variance_weight)
  {
    double position_variance =
        config.position_variance_weight*std::pow(2 * voxel_size, 2) / 12.0;
    tracker->setPositionVariance (position_variance);
  }
  //  }

  gate_distance = chi_map.find(config.gate_distance_probability) !=
      chi_map.end() ? chi_map[config.gate_distance_probability] :
      chi_map[0.999];
  tracker->setGateDistance (config.gate_distance_probability);
}

int
main(int argc, char** argv)
{

  ros::init(argc, argv, "skeleton_tracker");
  ros::NodeHandle nh("~");


  // Subscribers/Publishers:
  ros::Subscriber input_sub = nh.subscribe("input", 5, detection_cb);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>
      ("/tracker/skeleton_markers_array", 1);
  standard_skel_pub = nh.advertise<opt_msgs::StandardSkeletonTrackArray>
      ("/tracker/standard_skeleton_tracks", 1);
  standard_skel_markers_pub = nh.advertise<visualization_msgs::MarkerArray>
      ("/tracker/standard_skeleton_markers", 1);
  pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >
      ("/tracker/skeleton_history", 1);
  results_pub = nh.advertise<opt_msgs::SkeletonTrackArray>
      ("/tracker/skeleton_tracks", 100);
  skeleton_detection_centroid_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>
      ("/detector/skeleton_centroid_markers_array", 1);
  skeleton_detection_markers_pub = nh.advertise<visualization_msgs::MarkerArray>
      ("/detector/skeleton_markers_array", 1);
  detection_trajectory_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGBA> >
      ("/detector/skeleton_history", 1);
  alive_ids_pub = nh.advertise<opt_msgs::IDArray>("/tracker/alive_ids", 1);



  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  std::shared_ptr<ReconfigureServer> reconfigure_server_;

  tf_listener = new tf::TransformListener();

  // Read tracking parameters:
  nh.param("world_frame_id", world_frame_id, std::string("/odom"));

  nh.param("orientation/vertical", vertical, false);
  nh.param<bool>("extrinsic_calibration", extrinsic_calibration, false);

  nh.param("voxel_size", voxel_size, 0.06);

  double rate;
  nh.param("rate", rate, 30.0);

  //  double min_confidence;
  nh.param("min_confidence_initialization", min_confidence, -2.5); //0.0);

  double chi_value;
  nh.param("gate_distance_probability", chi_value, 0.9);

  nh.param("acceleration_variance", acceleration_variance, 1.0);

  nh.param("position_variance_weight", position_variance_weight, 1.0);

  bool detector_likelihood;
  nh.param("detector_likelihood", detector_likelihood, false);

  nh.param("velocity_in_motion_term", velocity_in_motion_term, false);

  double detector_weight;
  nh.param("detector_weight", detector_weight, -1.0);

  double motion_weight;
  nh.param("motion_weight", motion_weight, 0.5);

  double sec_before_old;
  nh.param("sec_before_old", sec_before_old, 3.6);

  double sec_before_fake;
  nh.param("sec_before_fake", sec_before_fake, 2.4);

  double sec_remain_new;
  nh.param("sec_remain_new", sec_remain_new, 1.2);

  int detections_to_validate;
  nh.param("detections_to_validate", detections_to_validate, 5);

  double haar_disp_ada_min_confidence,
      ground_based_people_detection_min_confidence;
  nh.param("haar_disp_ada_min_confidence",
           haar_disp_ada_min_confidence, -2.5); //0.0);
  nh.param("ground_based_people_detection_min_confidence",
           ground_based_people_detection_min_confidence, -2.5); //0.0);

  nh.param("swissranger", swissranger, false);

  nh.param("ground_based_people_detection_min_confidence_sr",
           min_confidence_detections_sr, -1.5);
  nh.param("min_confidence_initialization_sr", min_confidence_sr, -1.1);

  nh.param("history_pointcloud", output_history_pointcloud, false);
  nh.param("history_size", output_history_size, 1000);
  nh.param("markers", output_markers, true);
  nh.param("image_rgb", output_image_rgb, true);
  nh.param("tracking_results", output_tracking_results, true);

  nh.param("detection_debug", output_detection_results, true);
  nh.param("detection_history_size", detection_history_size, 1000);

  bool debug_mode;
  nh.param("debug_active", debug_mode, false);

  nh.param("calibration_refinement", calibration_refinement, false);
  nh.param("max_detection_delay", max_detection_delay, 3.0);

  double max_time_between_detections_d;
  nh.param("max_time_between_detections", max_time_between_detections_d, 10.0);
  max_time_between_detections_ = ros::Duration(max_time_between_detections_d);

  nh.param("remove_head_in_rviz", remove_head_in_rviz, true);

  // Read number of sensors in the network:
  int num_cameras = 1;
  if (extrinsic_calibration)
  {
    num_cameras = 0;
    XmlRpc::XmlRpcValue network;
    nh.getParam("network", network);
    for (unsigned i = 0; i < network.size(); i++)
    {
      num_cameras += network[i]["sensors"].size();
      for (unsigned j = 0; j < network[i]["sensors"].size(); j++)
      {
        std::string frame_id = network[i]["sensors"][j]["id"];
        last_received_detection_["/" + frame_id] = ros::Time(0);
      }
    }
  }

  // Set min_confidence_detections variable based on sensor type:
  if (swissranger)
    min_confidence_detections = ground_based_people_detection_min_confidence;
  else
    min_confidence_detections = haar_disp_ada_min_confidence;

  // Take chi square values with regards to the state dimension:
  //fillChiMap3D(chi_map, velocity_in_motion_term);
  fillChiMap(chi_map, velocity_in_motion_term);

  // Compute additional parameters:
  period = 1.0 / rate;
  gate_distance = chi_map.find(chi_value) != chi_map.end() ?
        chi_map[chi_value] : chi_map[0.999];

  double position_variance;
  //  position_variance = 3*std::pow(2 * voxel_size, 2) / 12.0; // DEFAULT
  position_variance = position_variance_weight*std::pow(2 * voxel_size, 2)
      / 12.0;
  std::vector<double> likelihood_weights;
  likelihood_weights.push_back(detector_weight*chi_map[0.999]/18.467);
  likelihood_weights.push_back(motion_weight);

  // Generate colors used to identify different cameras:
  generateColors(num_cameras, camera_colors);

  // Initialize point cloud containing detections trajectory:
  pcl::PointXYZRGB nan_point;
  nan_point.x = std::numeric_limits<float>::quiet_NaN();
  nan_point.y = std::numeric_limits<float>::quiet_NaN();
  nan_point.z = std::numeric_limits<float>::quiet_NaN();
  detection_history_pointcloud->points.resize(detection_history_size,
                                              nan_point);

  ros::Rate hz(num_cameras*rate);

  //  cv::namedWindow("TRACKER ", CV_WINDOW_NORMAL);

  // Initialize an instance of the Tracker object:
  tracker = new open_ptrack::tracking::SkeletonTracker(
        gate_distance,
        detector_likelihood,
        likelihood_weights,
        velocity_in_motion_term,
        min_confidence,
        min_confidence_detections,
        sec_before_old,
        sec_before_fake,
        sec_remain_new,
        detections_to_validate,
        period,
        position_variance,
        acceleration_variance,
        world_frame_id,
        debug_mode,
        vertical);

  starting_index = 0;

  // Set up dynamic reconfiguration
  ReconfigureServer::CallbackType f = boost::bind(&configCb, _1, _2);
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh));
  reconfigure_server_->setCallback(f);

  // If extrinsic calibration is not available:
  if (!extrinsic_calibration)
  { // Set fixed transformation from rgb frame and base_link
    // camera_rgb_optical_frame -> world
    tf::Vector3 fixed_translation(0, 0, 0);
    // camera_rgb_optical_frame -> world
    tf::Quaternion fixed_rotation(-0.5, 0.5, -0.5, -0.5);
    // world -> camera_rgb_optical_frame
    tf::Vector3 inv_fixed_translation(0.0, 0.0, 0);
    // world -> camera_rgb_optical_frame
    tf::Quaternion inv_fixed_rotation(-0.5, 0.5, -0.5, 0.5);
    world_to_camera_frame_transform = tf::Transform(fixed_rotation,
                                                    fixed_translation);
    camera_frame_to_world_transform = tf::Transform(inv_fixed_rotation,
                                                    inv_fixed_translation);
  }

  // Spin and execute callbacks:
  //  ros::spin();

  std::map<std::string, ros::Time> last_message;
  for (std::map<std::string, ros::Time>::const_iterator it =
       last_received_detection_.begin(), end = last_received_detection_.end();
       it != end; ++it)
    last_message[it->first] = ros::Time::now();

  // last time when the camera legend has been updated
  ros::Time last_camera_legend_update = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    for (std::map<std::string, ros::Time>::const_iterator
         it = last_received_detection_.begin(),
         end = last_received_detection_.end();
         it != end; ++it)
    {
      ros::Duration duration(now - it->second);
      if (duration > max_time_between_detections_)
      {
        if (it->second > ros::Time(0) and
            now - last_message[it->first] > max_time_between_detections_)
        {
          ROS_WARN_STREAM("[" << it->first << "] last detection was " <<
                          duration.toSec() << " seconds ago");
          last_message[it->first] = now;
        }
        else if (now - last_message[it->first] > max_time_between_detections_)
        {
          ROS_WARN_STREAM("[" << it->first
                          << "] still waiting for detection messages...");
          last_message[it->first] = now;
        }
      }

      // Update camera legend every second:
      // if more than one second passed since last update
      if ((now - last_camera_legend_update) > ros::Duration(1.0))
      { // update OpenCV image with a waitKey:
        cv::waitKey(1);
        last_camera_legend_update = now;
      }
    }
    hz.sleep();
  }

  return 0;
}

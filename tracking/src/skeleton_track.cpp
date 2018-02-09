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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it], Filippo Basso [filippo.basso@dei.unipd.it]
 *
 */

#include <ros/ros.h>

#include <open_ptrack/tracking/skeleton_track.h>

using namespace open_ptrack::bpe;

namespace open_ptrack
{
namespace tracking
{

geometry_msgs::Point
getGeomMsgsPoint(const rtpose_wrapper::Joint3DMsg& joint)
{
  geometry_msgs::Point p;
  p.x = joint.x;
  p.y = joint.y;
  p.z = joint.z;
  return p;
}

int SkeletonTrack::count = 0;

SkeletonTrack::SkeletonTrack(int id,
                             std::string frame_id, double position_variance,
                             double acceleration_variance, double period,
                             bool velocity_in_motion_term,
                             const std::vector<rtpose_wrapper::Joint3DMsg>& joints):
  Track(id, frame_id, position_variance, acceleration_variance, period,
        velocity_in_motion_term), all_joint_tracks_initialized_(false)
{
  joint_tracks_.resize(SkeletonJoints::SIZE);
  for(size_t i = 0; i < SkeletonJoints::SIZE; ++i)
  {
    joint_tracks_[i] = new Track3D(
          id,
          frame_id,
          position_variance,
          acceleration_variance,
          period,
          velocity_in_motion_term
          );
  }
  debug_count_ = -1;
}

SkeletonTrack::~SkeletonTrack()
{
  SkeletonTrack::count++;
}

bool
SkeletonTrack::anyNaNs(const std::vector<rtpose_wrapper::Joint3DMsg>& joints)
{
  for(size_t i = 0; i < SkeletonJoints::SIZE; ++i)
  {
    if(std::isnan(joints[i].x) or std::isnan(joints[i].y) or std::isnan(joints[i].z))
      return true;
  }
  return false;
}

void
SkeletonTrack::init(double x, double y, double z, double height, double distance,
                    open_ptrack::detection::DetectionSource* detection_source,
                    const std::vector<rtpose_wrapper::Joint3DMsg>& joints)
{
  Track::init(x,y,z,height,distance,detection_source);
  bool any_nan = anyNaNs(joints);
  any_nan? all_joint_tracks_initialized_ = false :
      all_joint_tracks_initialized_ = true;
  for(int i = 0, end = joint_tracks_.size();
      i != end and not any_nan; ++i)
  {
    const rtpose_wrapper::Joint3DMsg& bj = joints[i];
    joint_tracks_[i] -> init(bj.x, bj.y, bj.z, 10,
                             Eigen::Vector3d(bj.x, bj.y, bj.z).norm(),
                             detection_source
                             );
  }
//  if(SkeletonTrack::count == 0 && debug_count_ <= 0)
//    ROS_WARN_STREAM("TODO: SkeletonTrack: Initialize ");
  raw_joints_tmp_ = joints;

}

bool
SkeletonTrack::areJointsInitialized()
{
  return all_joint_tracks_initialized_;
}

void
SkeletonTrack::update(
    double x,
    double y,
    double z,
    double height,
    double distance,
    double data_assocation_score,
    double confidence,
    double min_confidence,
    double min_confidence_detections,
    open_ptrack::detection::DetectionSource* detection_source,
    const std::vector<rtpose_wrapper::Joint3DMsg>& joints,
    bool first_update)
{
  Track::update(x,y,z,height,distance,data_assocation_score,
                confidence,min_confidence,min_confidence_detections,
                detection_source,first_update);
  if(all_joint_tracks_initialized_)
  {
    for(int i = 0, end = SkeletonJoints::SIZE; i != end; ++i)
    {
      const rtpose_wrapper::Joint3DMsg& bj = joints[i];
      joint_tracks_[i] -> update(bj.x, bj.y, bj.z, 10,
                                 Eigen::Vector3d(bj.x, bj.y, bj.z).norm(), bj.confidence,
                                 bj.confidence - 1, bj.confidence + 1, 0,
                                 detection_source, first_update
                                 );
    }
  }
  else
  {
    bool any_nan = anyNaNs(joints);
    if (not any_nan)
    {
      all_joint_tracks_initialized_ = true;
      for(int i = 0, end = joint_tracks_.size();
          i != end; ++i)
      {
        const rtpose_wrapper::Joint3DMsg& bj = joints[i];
        joint_tracks_[i] -> init(bj.x, bj.y, bj.z, 10,
                                 Eigen::Vector3d(bj.x, bj.y, bj.z).norm(),
                                 detection_source
                                 );
        bool first_update = true;
        joint_tracks_[i] -> update(
              bj.x, bj.y, bj.z, 10,
              Eigen::Vector3d(bj.x, bj.y, bj.z).norm(), bj.confidence,
              bj.confidence - 1, bj.confidence + 1, 0,
              detection_source, first_update
              );
      }
    }
  }
//  if(SkeletonTrack::count == 0 && debug_count_ <= 0)
//    ROS_WARN_STREAM("SkeletonTrack TODO: apply update policy also "
//                    "to joint tracks");
  raw_joints_tmp_ = joints;
  debug_count_++;
}

void
SkeletonTrack::toMsg(opt_msgs::SkeletonTrack& track_msg, bool vertical)
{
  if(SkeletonTrack::count == 0 && debug_count_ <= 0)
    ROS_WARN_STREAM("SkeletonTrack TODO: Define the skel_track_msg");
  double _x,_y;
  filter_->getState(_x,_y);

  track_msg.id = id_;
  track_msg.x = _x;
  track_msg.y = _y;
  track_msg.height = z_;
  track_msg.distance = distance_;
  track_msg.age = age_;
  track_msg.color.a = 1.0;
  track_msg.color.r = color_(2);
  track_msg.color.g = color_(1);
  track_msg.color.b = color_(0);

  track_msg.confidence = - data_association_score_;   // minus for transforming distance into a sort of confidence
  track_msg.visibility = visibility_;

  Eigen::Vector3d top(_x, _y, z_ + (height_/2));
  Eigen::Vector3d bottom(_x, _y, z_ - (height_/2));
  top = detection_source_->transformToCam(top);
  bottom = detection_source_->transformToCam(bottom);
  if (not vertical)
  {
    track_msg.box_2D.height = int(std::abs((top - bottom)(1)));
    track_msg.box_2D.width = track_msg.box_2D.height / 2;
    track_msg.box_2D.x = int(top(0)) - track_msg.box_2D.height / 4;
    track_msg.box_2D.y = int(top(1));
  }
  else
  {
    track_msg.box_2D.width = int(std::abs((top - bottom)(0)));
    track_msg.box_2D.height = track_msg.box_2D.width / 2;
    track_msg.box_2D.x = int(top(0)) - track_msg.box_2D.width;
    track_msg.box_2D.y = int(top(1)) - track_msg.box_2D.width / 4;
  }
  track_msg.joints.resize(joint_tracks_.size());
  //  SkeletonTrack::bodyPoseMsgToTrackerMsg(track_msg);
  for(size_t i = 0; i < SkeletonJoints::SIZE; ++i)
  {
    Track3D* t = joint_tracks_[i];
    double jx, jy, jz;
    t -> getState(jx, jy, jz);
    opt_msgs::Track3D m;
    m.confidence = 1.0;
    m.x = jx;
    m.y = jy;
    m.z = jz;
    m.id = i;
    track_msg.joints[i] = m;
  }
}

//void
//SkeletonTrack::bodyPoseMsgToTrackerMsg(
//    opt_msgs::SkeletonTrack &msg)
//{
//  for(uint i = 0, end = joint_tracks_.size(); i != end; ++i)
//  {
//    opt_msgs::Track3D m;
//    const rtpose_wrapper::Joint3DMsg& j = joint_tracks_[i]->getState();
//    m.confidence = j.confidence;
//    m.x = j.x;
//    m.y = j.y;
//    m.z = j.z;
//    m.id = id_;
//    msg.joints[i] = m;
//  }
//}
bool
SkeletonTrack::isValid(const rtpose_wrapper::Joint3DMsg& joint)
{
  return std::isfinite(joint.x) and std::isfinite(joint.y)
      and std::isfinite(joint.z);
}

void
SkeletonTrack::createMarker(visualization_msgs::MarkerArray::Ptr& msg,
                            bool remove_head_in_rviz)
{
//  if(debug_count_ < 0 and SkeletonTrack::count == 0)
//    ROS_WARN_STREAM("SkeletonTrack TODO: reqrite createMarker with tracks");

  if(visibility_ == Track::NOT_VISIBLE)
    return;

  std_msgs::ColorRGBA color_random, red;
  color_random.a = red.a = 1.0;
  color_random.r = color_(2);
  color_random.g = color_(1);
  color_random.b = color_(0);
  red.r = 1.0;
  red.b = red.g = 0.0;



  ros::Time now(ros::Time::now());

  if (areJointsInitialized())
  {
    // Joint Markers
    for(int i = 0, end = SkeletonJoints::SIZE; i != end; ++i)
    {
      if(remove_head_in_rviz)
        if(i == SkeletonJoints::HEAD) continue;
      visualization_msgs::Marker joint_marker;
      joint_marker.header.frame_id = frame_id_;
      joint_marker.header.stamp = now;
      joint_marker.ns = "joints_valid";
      joint_marker.id = i + id_ * SkeletonJoints::SIZE * 2; //for visualizing both detection and tracks
      joint_marker.type = visualization_msgs::Marker::SPHERE;
      joint_marker.action = visualization_msgs::Marker::ADD;
      joint_marker.pose.position = joint_tracks_[i]->getState();
      joint_marker.pose.orientation.x = 0.0;
      joint_marker.pose.orientation.y = 0.0;
      joint_marker.pose.orientation.z = 0.0;
      joint_marker.pose.orientation.w = 1.0;
      joint_marker.scale.x = 0.06;
      joint_marker.scale.y = 0.06;
      joint_marker.scale.z = 0.06;
      joint_marker.color.r = color_(2);
      joint_marker.color.g = color_(1);
      joint_marker.color.b = color_(0);
      joint_marker.color.a = 1.0;

      joint_marker.lifetime = ros::Duration(0.2);


      //      geometry_msgs::Point p0;
      //      p0.x = p0.y = p0.z = 0;
      //      joint_marker.pose.position = p0;
      msg->markers.push_back(joint_marker);
    }
    // Link markers
    for(auto it = SkeletonLinks::LINKS.begin(), end = SkeletonLinks::LINKS.end();
        it != end; ++it)
    {
      if(remove_head_in_rviz)
        if(it->first == SkeletonJoints::HEAD
           or it->second == SkeletonJoints::HEAD)
          continue;
      const geometry_msgs::Point& p1 = joint_tracks_[it->first]->getState();
      const geometry_msgs::Point& p2 = joint_tracks_[it->second]->getState();
      //      if( isValid(p1)
      //          and
      //          isValid(p2))
      //      {
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = frame_id_;
      line_marker.header.stamp = now;
      line_marker.ns = "links_valid";
      line_marker.id = (it - SkeletonLinks::LINKS.begin())
          + (id_ - 1) * SkeletonLinks::LINKS.size() * 2;
      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::Marker::ADD;
      line_marker.pose.orientation.x = 0.0;
      line_marker.pose.orientation.y = 0.0;
      line_marker.pose.orientation.z = 0.0;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.1;
      line_marker.scale.y = 0.1;
      line_marker.scale.z = 0.1;
      line_marker.color.r = color_(2);
      line_marker.color.g = color_(1);
      line_marker.color.b = color_(0);
      line_marker.color.a = 1.0;

      line_marker.lifetime = ros::Duration(0.2);
      //        ROS_INFO_STREAM("P1: " << p1.x << "," << p1.y << "," << p1.z << " P2: " << p2.x << "," << p2.y << "," << p2.z);
      //        geometry_msgs::Point p0;
      //        p0.x = p0.y = p0.z = 0.0;
      line_marker.points.push_back(p1);
      line_marker.points.push_back(p2);
      line_marker.colors.push_back(color_random);
      line_marker.colors.push_back(color_random);
      msg->markers.push_back(line_marker);
      //      }
    }
  } // areJointsInitialized
  //  else
  //  {
  // Joint Markers
  for(int i = 0, end = SkeletonJoints::SIZE; i != end; ++i)
  {
    if ( not isValid(raw_joints_tmp_[i])) continue;
    if(remove_head_in_rviz)
      if(i == SkeletonJoints::HEAD) continue;
    visualization_msgs::Marker joint_marker;
    joint_marker.header.frame_id = frame_id_;
    joint_marker.header.stamp = now;
    joint_marker.ns = "joints_raw_" + raw_joints_tmp_[i].header.frame_id;
    joint_marker.id = i + id_ * SkeletonJoints::SIZE * 2
        + SkeletonJoints::SIZE;
    joint_marker.type = visualization_msgs::Marker::CUBE;
    joint_marker.action = visualization_msgs::Marker::ADD;
    joint_marker.pose.position = getGeomMsgsPoint(raw_joints_tmp_[i]);
    joint_marker.pose.orientation.x = 0.0;
    joint_marker.pose.orientation.y = 0.0;
    joint_marker.pose.orientation.z = 0.0;
    joint_marker.pose.orientation.w = 1.0;
    joint_marker.scale.x = 0.06;
    joint_marker.scale.y = 0.06;
    joint_marker.scale.z = 0.06;
    //      joint_marker.color.r = color_(2);
    //      joint_marker.color.g = color_(1);
    //      joint_marker.color.b = color_(0);
    joint_marker.color.r = 1.0;
    joint_marker.color.g = 0.0;
    joint_marker.color.b = 0.0;
    joint_marker.color.a = 1.0;

    joint_marker.lifetime = ros::Duration(0.2);

    //    geometry_msgs::Point p0;
    //    p0.x = p0.y = p0.z = 0;
    //    joint_marker.pose.position = p0;
    msg->markers.push_back(joint_marker);
  }
  // Link markers
  for(auto it = SkeletonLinks::LINKS.begin(), end = SkeletonLinks::LINKS.end();
      it != end; ++it)
  {
    const rtpose_wrapper::Joint3DMsg& p1 = raw_joints_tmp_[it->first];
    const rtpose_wrapper::Joint3DMsg& p2 = raw_joints_tmp_[it->second];
    if( isValid(p1)
        and
        isValid(p2))
    {
      if(remove_head_in_rviz)
        if(it->first == SkeletonJoints::HEAD
           or it->second == SkeletonJoints::HEAD)
          continue;
      visualization_msgs::Marker line_marker;
      line_marker.header.frame_id = frame_id_;
      line_marker.header.stamp = now;
      line_marker.ns = "links_raw_" + p1.header.frame_id;
      line_marker.id = (it - SkeletonLinks::LINKS.begin())
          + (id_ - 1) * SkeletonLinks::LINKS.size() * 2 + SkeletonLinks::LINKS.size();
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
      line_marker.color.g = 0.0;
      line_marker.color.b = 0.0;
      //        line_marker.color.r = color_(2);
      //        line_marker.color.g = color_(1);
      //        line_marker.color.b = color_(0);
      line_marker.color.a = 1.0;

      line_marker.lifetime = ros::Duration(0.2);
      //      geometry_msgs::Point p0;
      //      p0.x = p0.y = p0.z = 0.0f;
      line_marker.points.push_back(getGeomMsgsPoint(p1));
      line_marker.points.push_back(getGeomMsgsPoint(p2));
      line_marker.colors.push_back(red);
      line_marker.colors.push_back(red);
      msg->markers.push_back(line_marker);
    }
    //    }
  }

  // Track ID over head
  std::vector<geometry_msgs::Point> joints_tracks_tmp
      (joint_tracks_.size());
  for(uint i = 0; i < joint_tracks_.size(); ++i)
  {
    joints_tracks_tmp[i] = joint_tracks_[i]->getState();
  }
  Eigen::Vector3d world_centroid =
      open_ptrack::detection::SkeletonDetection::averageOverValidJoints
      (joints_tracks_tmp);
  double x = world_centroid[0];
  double y = world_centroid[1];
  double z = world_centroid[2];
  double variance = open_ptrack::detection::SkeletonDetection::varianceOverValidJoints(joints_tracks_tmp, world_centroid);

  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = frame_id_;
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "numbers";
  text_marker.id = id_;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  std::stringstream ss;
  ss << id_;
  text_marker.text = ss.str();
  text_marker.pose.position.x = x;
  text_marker.pose.position.y = y;
  text_marker.pose.position.z = z + 0.8;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.x = 0.34;
  text_marker.scale.y = 0.34;
  text_marker.scale.z = 0.34;
  text_marker.color.r = color_(2);
  text_marker.color.g = color_(1);
  text_marker.color.b = color_(0);
  text_marker.color.a = 1.0;
  text_marker.lifetime = ros::Duration(0.2);

  if (std::isnormal(fabs(x) + fabs(y) + fabs(z) + variance))
  {
//    std::cout << x << "," << y << "," << z << "," << variance << std::endl;
    msg->markers.push_back(text_marker);
  }
}

std::ostream&
operator<< (std::ostream& ss, const SkeletonTrack& s)
{
  double x, y;
  s.filter_->getState(x, y);
  ss << x << "," << y;
  return ss;
}


} /* namespace tracking */
} /* namespace open_ptrack */

/*
 * Software License Agreement (BSD License)
 *
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
 * Author: Matteo Munaro [matteo.munaro@dei.unipd.it]
 *
 */

#include <ros/ros.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/IDArray.h>
#include <opt_msgs/SkeletonTrackArray.h>
#include <opt_msgs/StandardSkeletonTrackArray.h>
#include <opt_msgs/PoseRecognitionArray.h>
#include <body_pose_estimation/skeleton_base.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <open_ptrack/opt_utils/udp_messaging.h>
#include <open_ptrack/opt_utils/json.h>

// Global variables:
int udp_buffer_length;  // UDP message buffer length
int udp_port;           // UDP port
std::string hostip;     // UDP host
int json_indent_size;   // indent size for JSON message
bool json_newline;      // use newlines (true) or not (false) in JSON messages
bool json_spacing;      // use spacing (true) or not (false) in JSON messages
bool json_use_tabs;     // use tabs (true) or not (false) in JSON messages
struct ComData udp_data;  // parameters for UDP messaging
open_ptrack::opt_utils::UDPMessaging udp_messaging(udp_data);   // instance of class UDPMessaging
ros::Time last_heartbeat_time;
double heartbeat_interval;

using namespace open_ptrack::bpe;

void
synchronizedCallback(
    const opt_msgs::SkeletonTrackArrayConstPtr& skel_track_msg,
    const opt_msgs::StandardSkeletonTrackArrayConstPtr& st_skel_msg,
    const opt_msgs::PoseRecognitionArrayConstPtr& pr_array_msg)
{
  // ROS_INFO_STREAM("Synchronized!");

//  /// Create JSON-formatted message:
  Jzon::Object root, header, stamp;
  Jzon::Array tracks;

//  /// Add header (84 characters):
  header.Add("seq", int(pr_array_msg->header.seq));
  stamp.Add("sec", int(pr_array_msg->header.stamp.sec));
  stamp.Add("nsec", int(pr_array_msg->header.stamp.nsec));
  header.Add("stamp", stamp); 

  std::string camera_name = skel_track_msg->header.frame_id;
    if (strcmp(camera_name.substr(0,1).c_str(), "/") == 0)  // Remove bar at the beginning
    {
      camera_name = camera_name.substr(1, camera_name.size() - 1);
    }
  
  header.Add("frame_id", camera_name);
  root.Add("header", header);

  // persons
  for (unsigned int i = 0; i < skel_track_msg->tracks.size(); i++)
  {
    Jzon::Object current_track;
    const opt_msgs::SkeletonTrack& t = skel_track_msg->tracks[i];
    const opt_msgs::StandardSkeletonTrack& st_t = st_skel_msg->tracks[i];
    const opt_msgs::PoseRecognition& pr = pr_array_msg->poses[i];
    current_track.Add("id", t.id);
    current_track.Add("height", t.height);
    if(std::isnan(st_t.orientation))
       current_track.Add("orientation", std::string("NaN"));
    else
       current_track.Add("orientation", st_t.orientation);
    current_track.Add("age", t.age);
    current_track.Add("predicted_pose_name",
                      pr.best_prediction_result.pose_name);
    current_track.Add("predicted_pose_id",
                      pr.best_prediction_result.pose_id);
    current_track.Add("prediction_score",
                      pr.best_prediction_result.score);
    // poses
    Jzon::Array poses;
    for (size_t p_id = 0, p_end = pr.gallery_poses.size(); p_id != p_end;
         ++p_id)
    {
      Jzon::Object pose;
      const opt_msgs::PosePredictionResult& ppr = pr.gallery_poses[p_id];
      pose.Add("pose_name", ppr.pose_name);
      pose.Add("pose_id", ppr.pose_id);
      if(std::isnan(ppr.score))
       pose.Add("prediction_score", std::string("nan"));
      else
       pose.Add("prediction_score", ppr.score);
      poses.Add(pose);
    }
    current_track.Add("poses", poses);

    // joints
    Jzon::Object joints;
    for (size_t j_id = 0, j_end = t.joints.size(); j_id != j_end;
         ++j_id)
    {
      const opt_msgs::Track3D& j = t.joints[j_id];
      const std::string& s = SkeletonBase::JOINT_NAMES[j_id];
      Jzon::Object jj;
      if(std::isnan(j.x))
       jj.Add("x", std::string("nan"));
      else
       jj.Add("x", j.x);
      if(std::isnan(j.y))
       jj.Add("y", std::string("nan"));
      else
       jj.Add("y", j.y);
      if(std::isnan(j.z))
       jj.Add("z", std::string("nan"));
      else
       jj.Add("z", j.z);

      if(std::isnan(j.confidence))
       jj.Add("confidence", std::string("nan"));
      else
       jj.Add("confidence", j.confidence);

      joints.Add(s, jj);
    }
    current_track.Add("joints", joints);

    tracks.Add(current_track);
  }
  root.Add("pose_tracks", tracks);

  /// Convert JSON object to string:
  Jzon::Format message_format = Jzon::StandardFormat;
  message_format.indentSize = json_indent_size;
  message_format.newline = json_newline;
  message_format.spacing = json_spacing;
  message_format.useTabs = json_use_tabs;
  Jzon::Writer writer(root, message_format);
  writer.Write();
  std::string json_string = writer.GetResult();
  //  std::cout << "String sent: " << json_string << std::endl;

  /// Copy string to message buffer:
  udp_data.si_num_byte_ = json_string.length()+1;
  char buf[udp_data.si_num_byte_];
  for (unsigned int i = 0; i < udp_data.si_num_byte_; i++)
  {
    buf[i] = 0;
  }
  sprintf(buf, "%s", json_string.c_str());
  udp_data.pc_pck_ = buf;         // buffer where the message is written

  /// Send message:
  udp_messaging.sendFromSocketUDP(&udp_data);
}


typedef unsigned long uint32;
// convert a string represenation of an IP address into its numeric equivalent
static uint32 Inet_AtoN(const char * buf)
{

  uint32 ret = 0;
  int shift = 24;  // fill out the MSB first
  bool startQuad = true;
  while((shift >= 0)&&(*buf))
  {
    if (startQuad)
    {
      unsigned char quad = (unsigned char) atoi(buf);
      ret |= (((uint32)quad) << shift);
      shift -= 8;
    }
    startQuad = (*buf == '.');
    buf++;
  }
  return ret;
}

int
main(int argc, char **argv)
{
  // Initialization:
  ros::init(argc, argv, "ros2udp_converter");
  ros::NodeHandle nh("~");

  // Read input parameters:
  nh.param("udp/port", udp_port, 21234);
  nh.param("udp/hostip", hostip, std::string("127.0.0.1"));
  nh.param("udp/buffer_length", udp_buffer_length, 2048);
  nh.param("json/indent_size", json_indent_size, 0);
  nh.param("json/newline", json_newline, false);
  nh.param("json/spacing", json_spacing, false);
  nh.param("json/use_tabs", json_use_tabs, false);
  nh.param("json/heartbeat_interval", heartbeat_interval, 0.25);

  // ROS subscriber:
  
  message_filters::Subscriber<opt_msgs::SkeletonTrackArray>
      skel_track_array_sub(nh, "/tracker/skeleton_tracks", 5);
  message_filters::Subscriber<opt_msgs::StandardSkeletonTrackArray>
      standard_skel_track_array_sub(nh, "/tracker/standard_skeleton_tracks", 5);
  message_filters::Subscriber<opt_msgs::PoseRecognitionArray>
      pose_recognition_array_sub(nh, "/recognizer/poses", 5);
  // message_filters::TimeSynchronizer<opt_msgs::SkeletonTrackArray,
  //     opt_msgs::StandardSkeletonTrackArray, opt_msgs::PoseRecognitionArray>
  //     sync(skel_track_array_sub, standard_skel_track_array_sub,
  //          pose_recognition_array_sub, 5);
  // sync.registerCallback(boost::bind(&synchronizedCallback, _1, _2, _3));
  typedef message_filters::sync_policies::ApproximateTime
      <opt_msgs::SkeletonTrackArray,
      opt_msgs::StandardSkeletonTrackArray,
      opt_msgs::PoseRecognitionArray> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10),
                                                    skel_track_array_sub,
                                                    standard_skel_track_array_sub,
                                                    pose_recognition_array_sub);
  sync_.registerCallback(boost::bind(&synchronizedCallback, _1, _2, _3));




  // Initialize UDP parameters:
  char buf[0];
  udp_data.si_port_ = udp_port;      // port
  udp_data.si_retry_ = 1;
  udp_data.si_num_byte_ = udp_buffer_length; // number of bytes to write (2048 -> about 30 tracks)
  udp_data.pc_pck_ = buf;         // buffer where the message is written
  udp_data.si_timeout_ = 4;
  udp_data.sj_addr_ = Inet_AtoN(hostip.c_str());

  /// Create object for UDP messaging:
  udp_messaging = open_ptrack::opt_utils::UDPMessaging(udp_data);

  /// Create client socket:
  udp_messaging.createSocketClientUDP(&udp_data);

  // Execute callbacks:
  ros::spin();

  // Close socket:
  udp_messaging.closeSocketUDP(&udp_data);

  return 0;
}

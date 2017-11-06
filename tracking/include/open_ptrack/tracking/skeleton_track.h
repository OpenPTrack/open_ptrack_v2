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

#ifndef OPEN_PTRACK_TRACKING_SKELETON_TRACK_H_
#define OPEN_PTRACK_TRACKING_SKELETON_TRACK_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <open_ptrack/opt_utils/conversions.h>
#include <open_ptrack/tracking/kalman_filter3d.h>
#include <open_ptrack/bayes/bayesFlt.hpp>
#include <open_ptrack/detection/detection_source.h>
#include <open_ptrack/detection/skeleton_detection.h>
#include <open_ptrack/tracking/track.h>
#include <open_ptrack/tracking/track3d.h>
#include <open_ptrack/tracking/tracker3d.h>
#include <opt_msgs/Track3D.h>
#include <opt_msgs/SkeletonTrack.h>
#include <memory>
#include <body_pose_estimation/skeleton_base.h>

namespace open_ptrack
{
namespace tracking
{

/** \brief SkeletonTrack represents information about a track (or target) */
class SkeletonTrack : public Track
{
private:
  bool
  isValid(const rtpose_wrapper::Joint3DMsg& joint);
protected:

  std::vector<Track3D*> joint_tracks_;
  std::vector<rtpose_wrapper::Joint3DMsg> raw_joints_tmp_;
  bool all_joint_tracks_initialized_;

  static int count;
  int debug_count_;

public:

  /** \brief Constructor. */
  SkeletonTrack(int id,
                std::string frame_id, double position_variance,
                double acceleration_variance, double period,
                bool velocity_in_motion_term,
                const std::vector<rtpose_wrapper::Joint3DMsg>& joints);

  /** \brief Destructor. */
  virtual ~SkeletonTrack();

  bool
  anyNaNs(const std::vector<rtpose_wrapper::Joint3DMsg>& joints);

  void
  update(double x,
         double y, double z,
         double height,
         double distance,
         double data_assocation_score,
         double confidence,
         double min_confidence,
         double min_confidence_detections,
         open_ptrack::detection::DetectionSource* detection_source,
         const std::vector<rtpose_wrapper::Joint3DMsg>& joints,
         bool first_update = false);

  void
  createJointMarker(
      visualization_msgs::MarkerArray::Ptr& msg,
      const Eigen::Vector3f& color);

  void
  toMsg(opt_msgs::SkeletonTrack& track_msg,
        bool vertical);

  void
  init(double x, double y, double z, double height, double distance,
       open_ptrack::detection::DetectionSource* detection_source,
       const std::vector<rtpose_wrapper::Joint3DMsg>& joints);

  void
  createMarker(visualization_msgs::MarkerArray::Ptr& msg,
               bool remove_head_in_rviz = false);

  void
  createSkeletonMarker(visualization_msgs::MarkerArray::Ptr& msg);

  void
  bodyPoseMsgToTrackerMsg(opt_msgs::SkeletonTrack &msg);

  static void
  bodyPoseMsgToMarker(const std::vector<rtpose_wrapper::Joint3DMsg>& joints,
                      visualization_msgs::Marker &msg);

  bool
  areJointsInitialized();

  friend std::ostream&
  operator<< (std::ostream& ss, const SkeletonTrack& s);
};

} /* namespace tracking */
} /* namespace open_ptrack */
#endif /* OPEN_PTRACK_TRACKING_TRACK_H_ */

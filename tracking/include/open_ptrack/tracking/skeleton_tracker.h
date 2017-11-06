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

#ifndef OPEN_PTRACK_TRACKING_SKELETON_TRACKER_H_
#define OPEN_PTRACK_TRACKING_SKELETON_TRACKER_H_

#include <open_ptrack/detection/skeleton_detection.h>
#include <open_ptrack/tracking/skeleton_track.h>
#include <open_ptrack/tracking/munkres.h>
#include <open_ptrack/tracking/tracker.h>
#include <opt_msgs/SkeletonTrackArray.h>
#include <opt_msgs/SkeletonTrack.h>
#include <opt_msgs/IDArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace open_ptrack
{
namespace tracking
{


/** \brief Tracker performs tracking-by-detection */
class SkeletonTracker : public Tracker
{
  typedef std::vector<open_ptrack::detection::SkeletonDetection>
  SkeletonDetectionVector;

protected:

  static int count;

  ros::Publisher debug_pub_;

  /** \brief List of all active tracks */
  std::list<open_ptrack::tracking::SkeletonTrack*> tracks_;

  /** \brief List of lost tracks */
  std::list<open_ptrack::tracking::SkeletonTrack*> lost_tracks_;

  /** \brief List of tracks with Status = NEW */
  std::list<open_ptrack::tracking::SkeletonTrack*> new_tracks_;

  /** \brief List of current detections */
  std::vector<open_ptrack::detection::SkeletonDetection> detections_;

  /** \brief List of current detections not associated to any track */
  std::list<open_ptrack::detection::SkeletonDetection> unassociated_detections_;

  void
  createDistanceMatrix();

  void
  updateDetectedTracks();

  void
  createNewTracks();

  void
  createCostMatrix();

  void
  fillUnassociatedDetections();

  int
  createNewTrack(open_ptrack::detection::SkeletonDetection& detection);

  void
  renderJointsAndLines(
      visualization_msgs::Marker& lines,
      visualization_msgs::Marker& joints,
      opt_msgs::SkeletonTrack &skeleton
      );

public:

  /** \brief Constructor */
  SkeletonTracker(double gate_distance, bool detector_likelihood,
                  std::vector<double> likelihood_weights,
                  bool velocity_in_motion_term,
                  double min_confidence,
                  double min_confidence_detections,
                  double sec_before_old, double sec_before_fake,
                  double sec_remain_new, int detections_to_validate,
                  double period, double position_variance,
                  double acceleration_variance,
                  std::string world_frame_id, bool debug_mode,
                  bool vertical):
    Tracker(gate_distance, detector_likelihood, likelihood_weights,
            velocity_in_motion_term, min_confidence,
            min_confidence_detections, sec_before_old, sec_before_fake,
            sec_remain_new, detections_to_validate, period,
            position_variance, acceleration_variance,
            world_frame_id, debug_mode, vertical)
  {  }
  /** \brief Destructor */
  virtual ~SkeletonTracker();

  /** \brief newFrame method
   *
   */
  void
  newFrame(const SkeletonDetectionVector& detections);

  void
  toMsg(opt_msgs::SkeletonTrackArray::Ptr& skel_track_array);

  void
  updateTracks();

  void
  toMarkerArray(visualization_msgs::MarkerArray::Ptr& msg,
                bool remove_head_in_rviz = false);

  void
  toSkeletonMarkerArray(visualization_msgs::MarkerArray::Ptr& msg);

};

} /* namespace tracking */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_TRACKING_TRACKER_H_ */

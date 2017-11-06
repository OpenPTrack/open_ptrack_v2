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

#include <opencv2/opencv.hpp>

#include <open_ptrack/tracking/skeleton_tracker.h>

namespace open_ptrack
{
namespace tracking
{

int SkeletonTracker::count = 0;

SkeletonTracker::~SkeletonTracker()
{
  SkeletonTracker::count++;
}

void
SkeletonTracker::newFrame(const SkeletonDetectionVector& detections)
{
  detections_.clear();
  unassociated_detections_.clear();
  lost_tracks_.clear();
  new_tracks_.clear();
  detections_ = detections;

  ros::Time current_detections_time = detections_[0].getSource()->getTime();

  for(std::list<open_ptrack::tracking::SkeletonTrack*>::iterator
      it = tracks_.begin(), end = tracks_.end();
      it != end;)
  {
    open_ptrack::tracking::SkeletonTrack* t = *it;
    bool deleted = false;

    if(((t->getVisibility() == SkeletonTrack::NOT_VISIBLE && (t->getSecFromLastHighConfidenceDetection(current_detections_time)) >= sec_before_old_)
        || (!t->isValidated() && t->getSecFromFirstDetection(current_detections_time) >= sec_before_fake_)))
    {
      ROS_INFO_STREAM("first test: " << (t->getVisibility() == SkeletonTrack::NOT_VISIBLE && (t->getSecFromLastHighConfidenceDetection(current_detections_time)) >= sec_before_old_));
      ROS_INFO_STREAM("is not validated and > sec_fake " << (!t->isValidated() && t->getSecFromFirstDetection(current_detections_time) >= sec_before_fake_));
      if (debug_mode_)
      {
        std::cout << "Track " << t->getId() << " DELETED" << std::endl;
      }
      delete t;
      it = tracks_.erase(it);
      deleted = true;
    }
    else if(!t->isValidated() && t->getUpdatesWithEnoughConfidence() == detections_to_validate_)
    {
      t->validate();
      if (debug_mode_)
      {
        std::cout << "Track " << t->getId() << " VALIDATED" << std::endl;
      }
    }
    else if(t->getStatus() == SkeletonTrack::NEW && t->getSecFromFirstDetection(current_detections_time) >= sec_remain_new_)
    {
      t->setStatus(SkeletonTrack::NORMAL);
      if (debug_mode_)
      {
        std::cout << "Track " << t->getId() << " set to NORMAL" << std::endl;
      }
    }

    if(!deleted)
    {
      if(t->getStatus() == SkeletonTrack::NEW
         && t->getVisibility() == SkeletonTrack::VISIBLE)
      {
        new_tracks_.push_back(t);
      }
      if(t->getVisibility() == SkeletonTrack::NOT_VISIBLE)
      {
        lost_tracks_.push_back(t);
      }
      it++;
    }
  }
}

void
SkeletonTracker::toMsg(opt_msgs::SkeletonTrackArrayPtr &skel_track_array)
{
  for(std::list<open_ptrack::tracking::SkeletonTrack*>::iterator
      it = tracks_.begin(), end = tracks_.end(); it != end; it++)
  {
    opt_msgs::SkeletonTrack track;
    (*it)->toMsg(track, vertical_);
    skel_track_array->tracks.push_back(track);
  }
}

void
SkeletonTracker::renderJointsAndLines(
    visualization_msgs::Marker& lines,
    visualization_msgs::Marker& joints,
    opt_msgs::SkeletonTrack &skeleton
    )
{}

void
SkeletonTracker::updateTracks()
{
  createDistanceMatrix();
  createCostMatrix();

  // Solve Global Nearest Neighbor problem:
  Munkres munkres;
  cost_matrix_ = munkres.solve(cost_matrix_, false);	// rows: targets (tracks), cols: detections

  updateDetectedTracks();
  fillUnassociatedDetections();
  updateLostTracks();
  createNewTracks();
}

void
SkeletonTracker::createCostMatrix()
{
  cost_matrix_ = distance_matrix_.clone();
  for(int i = 0; i < distance_matrix_.rows; i++)
  {
    for(int j = 0; j < distance_matrix_.cols; j++)
    {
      if(distance_matrix_(i, j) > gate_distance_)
        cost_matrix_(i, j) = 1000000.0;
    }
  }
}

void
SkeletonTracker::fillUnassociatedDetections()
{
  // Fill a list with detections not associated to any track:
  if(cost_matrix_.cols == 0 && detections_.size() > 0)
  {
    for(size_t measure = 0; measure < detections_.size(); measure++)
      unassociated_detections_.push_back(detections_[measure]);
  }
  else
  {
    for(int measure = 0; measure < cost_matrix_.cols; measure++)
    {
      bool associated = false;
      for(int track = 0; track < cost_matrix_.rows; track++)
      {
        if(cost_matrix_(track, measure) == 0.0)
        {
          if(distance_matrix_(track, measure) > gate_distance_)
            break;
          associated = true;
        }
      }
      if(!associated/* && detections_[measure].getConfidence() > min_confidence_*/)
      {
        unassociated_detections_.push_back(detections_[measure]);
      }
    }
  }
}

void
SkeletonTracker::createDistanceMatrix()
{
  distance_matrix_ = cv::Mat_<double>(tracks_.size(), detections_.size());
  int track = 0;
  for(std::list<SkeletonTrack*>::const_iterator it = tracks_.begin(),
      end = tracks_.end(); it != end; it++)
  {
    SkeletonTrack* t = *it;
    int measure = 0;
    for(std::vector<open_ptrack::detection::SkeletonDetection>::iterator
        dit = detections_.begin(), dend = detections_.end(); dit != dend; dit++)
    {
      double detector_likelihood;

      // Compute detector likelihood:
      if (detector_likelihood_)
      {
        detector_likelihood = dit->getConfidence();
        //				detector_likelihood = log((dit->getConfidence() + 3) / 6);
      }
      else
      {
        detector_likelihood = 0;
      }

      // Compute motion likelihood:
      double motion_likelihood = t->getMahalanobisDistance(
            dit->getWorldCentroid()(0),
            dit->getWorldCentroid()(1),
            dit->getSource()->getTime());

      // Compute joint likelihood and put it in the distance matrix:

      distance_matrix_(track, measure++) =
          likelihood_weights_[0] * detector_likelihood +
          likelihood_weights_[1] * motion_likelihood;

      // Remove NaN and inf:
      if (std::isnan(distance_matrix_(track, measure-1))
          or (not std::isfinite(distance_matrix_(track, measure-1))))
        distance_matrix_(track, measure-1) = 2*gate_distance_;
    }
    track++;
  }
}

void
SkeletonTracker::updateDetectedTracks()
{
  // Iterate over every track:
  int track = 0;
  for(std::list<open_ptrack::tracking::SkeletonTrack*>::iterator
      it = tracks_.begin(), end = tracks_.end(); it != end; it++)
  {
    bool updated = false;
    open_ptrack::tracking::SkeletonTrack* t = *it;

    for(int measure = 0; measure < cost_matrix_.cols; measure++)
    {
      // If a detection<->track association has been found:
      if(cost_matrix_(track, measure) == 0.0 &&
         distance_matrix_(track, measure) <= gate_distance_)
      {
        open_ptrack::detection::SkeletonDetection& d = detections_[measure];

        // If the detection has enough confidence in the current
        // frame or in a recent past:
        if ((t->getLowConfidenceConsecutiveFrames() < 10)
            or (d.getConfidence() >
                ((min_confidence_ + min_confidence_detections_)/2)))
        {
          // Update track with the associated detection:
          bool first_update = false;
          t->update(d.getWorldCentroid()(0),
                    d.getWorldCentroid()(1),
                    d.getWorldCentroid()(2),
                    d.getHeight(),
                    d.getDistance(), distance_matrix_(track, measure),
                    d.getConfidence(), min_confidence_,
                    min_confidence_detections_,
                    d.getSource(), d.getSkeletonMsg().joints,
                    first_update
                    );

          t->setVisibility(d.isOccluded() ? SkeletonTrack::OCCLUDED :
                                            SkeletonTrack::VISIBLE);
          updated = true;
          break;
        }
      }
    }
    if(!updated)
    {
      if(t->getVisibility() != SkeletonTrack::NOT_VISIBLE)
      {
        t->setVisibility(SkeletonTrack::NOT_VISIBLE);
      }
    }
    track++;
  }
}

void
SkeletonTracker::createNewTracks()
{
  for(std::list<open_ptrack::detection::SkeletonDetection>::iterator
      dit = unassociated_detections_.begin(),
      dend = unassociated_detections_.end();
      dit != dend; dit++)
  {
    createNewTrack(*dit);
  }
}

int
SkeletonTracker::createNewTrack(open_ptrack::detection::SkeletonDetection&
                                detection)
{
  if(SkeletonTracker::count == 0)
    ROS_INFO_STREAM("TODO: SkeletonTracker createNewTrack: min_confidence check (createNewTrack)");
  //  ROS_INFO_STREAM("detection conf: " << detection.getConfidence());
  //  if(detection.getConfidence() < min_confidence_)
  //    return -1;

  open_ptrack::tracking::SkeletonTrack* t;
  t = new open_ptrack::tracking::SkeletonTrack(
        ++tracks_counter_,
        world_frame_id_,
        position_variance_,
        acceleration_variance_,
        period_,
        velocity_in_motion_term_,
        detection.getSkeletonMsg().joints);

  t->init(detection.getWorldCentroid()(0), detection.getWorldCentroid()(1),
          detection.getWorldCentroid()(2),
          detection.getHeight(),
          detection.getDistance(), detection.getSource(),
          detection.getSkeletonMsg().joints);

  bool first_update = true;
  t->update(detection.getWorldCentroid()(0),
            detection.getWorldCentroid()(1),
            detection.getWorldCentroid()(2),
            detection.getHeight(), detection.getDistance(), 0.0,
            detection.getConfidence(), min_confidence_,
            min_confidence_detections_, detection.getSource(),
            detection.getSkeletonMsg().joints,
            first_update
            );

  ROS_INFO_STREAM("Created " << t->getId());

  tracks_.push_back(t);
  return tracks_counter_;
}

void
SkeletonTracker::toMarkerArray(visualization_msgs::MarkerArray::Ptr &msg,
                               bool remove_head_in_rviz)
{
  for(std::list<open_ptrack::tracking::SkeletonTrack*>::iterator
      it = tracks_.begin(); it != tracks_.end(); it++)
  {
    open_ptrack::tracking::SkeletonTrack* t = *it;
    if(t->getVisibility() == open_ptrack::tracking::SkeletonTrack::OCCLUDED
       or
       t->getVisibility() == open_ptrack::tracking::SkeletonTrack::NOT_VISIBLE)
      continue;
    t->createMarker(msg, remove_head_in_rviz);
  }
}


} /* namespace tracking */
} /* namespace open_ptrack */

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

#ifndef OPEN_PTRACK_DETECTION_SKELETON_DETECTION_H_
#define OPEN_PTRACK_DETECTION_SKELETON_DETECTION_H_

#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <open_ptrack/detection/detection.h>
#include <opt_msgs/Detection.h>
#include <open_ptrack/detection/detection_source.h>
#include <rtpose_wrapper/SkeletonMsg.h>
#include <numeric>

namespace open_ptrack
{
namespace detection
{
/** \brief Detection represents information about a people detection */
class SkeletonDetection : public Detection
{

protected:
  /** \brief ROS message containing skeleton detection information */
  rtpose_wrapper::SkeletonMsg detection_msg_;

  static int debug_count;
  /**
         * \brief Compute the centroid to use for data/track association
         *
         * \return the centroid.
         */
  Eigen::Vector3d computeCentroid() const;

public:

  /** \brief Constructor. */
  SkeletonDetection(const rtpose_wrapper::SkeletonMsg &detection,
                    open_ptrack::detection::DetectionSource* source);
  bool
  isValidJoint(const rtpose_wrapper::Joint3DMsg& joint) const;

  inline rtpose_wrapper::SkeletonMsg
  getSkeletonMsg() const { return detection_msg_; }

  /** \brief Destructor. */
  virtual ~SkeletonDetection();

  double getConfidence() const;

  double
  getDistance() const;

  bool
  isOccluded() const;

  void
  setConfidence(double confidence);

  virtual void
  refineSkeletonJoints(const Eigen::Matrix4d& registration_matrix);

  friend std::ostream& operator<<(std::ostream& ss, const SkeletonDetection &s);

  static Eigen::Vector3d
  averageOverValidJoints(const std::vector<rtpose_wrapper::Joint3DMsg> &joints);
  static Eigen::Vector3d
  averageOverValidJoints(const std::vector<geometry_msgs::Point> &joints);

  static double
  varianceOverValidJoints(const std::vector<geometry_msgs::Point>& joints, const Eigen::Vector3d av);

};
} /* namespace detection */
} /* namespace open_ptrack */

#endif /* OPEN_PTRACK_DETECTION_DETECTION_H_ */

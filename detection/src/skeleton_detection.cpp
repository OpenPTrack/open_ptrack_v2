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

#include <open_ptrack/detection/skeleton_detection.h>

namespace open_ptrack
{
namespace detection
{

Eigen::Vector3d
SkeletonDetection::averageOverValidJoints(
    const std::vector<rtpose_wrapper::Joint3DMsg>& joints)
{
  Eigen::Vector3d v(0.0,0.0,0.0);
  auto acc_lambda = [](Eigen::Vector3d m,
      const rtpose_wrapper::Joint3DMsg& m1)
  {
    return std::isfinite(m1.x) and std::isfinite(m1.y) and std::isfinite(m1.z)?
          m + Eigen::Vector3d(m1.x,m1.y,m1.z):
          m;
  };
  auto count_lambda = [](const rtpose_wrapper::Joint3DMsg& m) {
    return not ((fabs(m.x) < 0.01 and fabs(m.y) < 0.01 and fabs(m.z) < 0.01)
                or not (std::isfinite(m.x) and std::isfinite(m.y)
                        and std::isfinite(m.z))
                );};
  v = std::accumulate(joints.begin(),
                      joints.end(),
                      Eigen::Vector3d(0.0,0.0,0.0),
                      acc_lambda);
  uint count = std::count_if(joints.begin(),
                             joints.end(),
                             count_lambda);
  return count == 0 ? Eigen::Vector3d(0.0,0.0,0.0) : Eigen::Vector3d(v / count);

}

double
SkeletonDetection::varianceOverValidJoints(
    const std::vector<geometry_msgs::Point>& joints, const Eigen::Vector3d av)
{
  Eigen::Vector3f diff(.0, .0, .0);

  for (uint i = 0, n = joints.size(); i != n; ++i)
  {
    Eigen::Vector3f loc_diff(fabs(av[0] - joints[i].x), fabs(av[1] - joints[i].y),
        fabs(av[2] - joints[i].z));
    diff += loc_diff;
  }
  diff *= 0.333;

  return diff.sum();
}

Eigen::Vector3d
SkeletonDetection::averageOverValidJoints(
    const std::vector<geometry_msgs::Point>& joints)
{
  Eigen::Vector3d v(0.0,0.0,0.0);
  auto acc_lambda = [](Eigen::Vector3d m,
      const geometry_msgs::Point& m1)
  {
    return std::isnormal(m1.x) and std::isnormal(m1.y) and std::isnormal(m1.z)?
          m + Eigen::Vector3d(m1.x,m1.y,m1.z):
          m;
  };
  auto count_lambda = [](const geometry_msgs::Point& m) {
    return not ((fabs(m.x) < 0.01 and fabs(m.y) < 0.01 and fabs(m.z) < 0.01)
                or not (std::isfinite(m.x) and std::isfinite(m.y)
                        and std::isfinite(m.z))
                );};
  v = std::accumulate(joints.begin(),
                      joints.end(),
                      Eigen::Vector3d(0.0,0.0,0.0),
                      acc_lambda);
  uint count = std::count_if(joints.begin(),
                             joints.end(),
                             count_lambda);
  return count == 0 ? Eigen::Vector3d(0.0,0.0,0.0) : Eigen::Vector3d(v / count);

}

bool
SkeletonDetection::isValidJoint(const rtpose_wrapper::Joint3DMsg& joint) const
{
  return std::isfinite(joint.x)
       and std::isfinite(joint.y)
       and std::isfinite(joint.z);
}

SkeletonDetection::SkeletonDetection
(const rtpose_wrapper::SkeletonMsg &detection,
 open_ptrack::detection::DetectionSource* source):
  Detection (opt_msgs::Detection(), source)
{
  detection_msg_ = detection;
  source_ = source;

  Eigen::Vector3d v = computeCentroid();
  world_centroid_ = source->transform(v);

  for(auto it = detection_msg_.joints.begin(),
      end = detection_msg_.joints.end(); it !=end; ++it)
  {
    int index = (it - detection_msg_.joints.begin());
    Eigen::Vector3d tmp = source->transform(Eigen::Vector3d
                                            (it->x, it->y, it->z));
    it->x = tmp(0);
    it->y = tmp(1);
    it->z = tmp(2);
  }

}

Eigen::Vector3d
SkeletonDetection::computeCentroid() const
{
  Eigen::Vector3d v;

  //  switch(detection_msg_.skeleton_type)
  //  {
  //  case rtpose_wrapper::SkeletonMsg::MPI: //MPI
  //  {
  //    v = averageOverValidJoints(detection_msg_.joints);
  //    break;
  //  }
  //  case rtpose_wrapper::SkeletonMsg::COCO: //COCO
  //  {
  //    // barycenter of joints 1, 8 and 11
  //    if(not isValidJoint(detection_msg_.joints[1])
  //       or not isValidJoint(detection_msg_.joints[8])
  //       or not isValidJoint(detection_msg_.joints[11]))
  //    {
  //      ROS_INFO_STREAM("WARNING: Centroid obtained from at least "
  //                      "one non-valid joint");
  //      v = averageOverValidJoints(detection_msg_.joints);
  //    }
  //    double mul_factor = 0.333f;
  //    v(0) = (detection_msg_.joints[1].x +
  //        detection_msg_.joints[8].x + detection_msg_.joints[11].x) * mul_factor;
  //    v(1) = (detection_msg_.joints[1].y +
  //        detection_msg_.joints[8].y + detection_msg_.joints[11].y) * mul_factor;
  //    v(2) = (detection_msg_.joints[1].z +
  //        detection_msg_.joints[8].z + detection_msg_.joints[11].z) * mul_factor;
  //    break;
  //  }
  //    // default = average between valid joints
  //  default:
  //  {
  v = averageOverValidJoints(detection_msg_.joints);
  //  }
  //  } // switch
  return v;
}

void
SkeletonDetection::refineSkeletonJoints(const
                                        Eigen::Matrix4d& registration_matrix)
{
  for(auto it = detection_msg_.joints.begin(),
      end = detection_msg_.joints.end(); it != end; ++it)
  {
    Eigen::Vector4d refined = registration_matrix
        * Eigen::Vector4d(it->x, it->y, it->z, 1.0);
    it->x = refined(0); it->y = refined(1); it->z = refined(2);
  }
}

std::ostream&
operator<<(std::ostream& ss, const SkeletonDetection& s)
{
  ss << s.getWorldCentroid().transpose();
  return ss;
}

int SkeletonDetection::debug_count = 0;
SkeletonDetection::~SkeletonDetection()
{
  SkeletonDetection::debug_count++;
}

double
SkeletonDetection::getConfidence() const
{
  return detection_msg_.confidence;
}

double
SkeletonDetection::getDistance() const
{
  return detection_msg_.distance;
}

bool
SkeletonDetection::isOccluded() const
{
  return detection_msg_.occluded;
}

void
SkeletonDetection::setConfidence(double confidence)
{
  detection_msg_.confidence = confidence;
}

} /* namespace detection */
} /* namespace open_ptrack */

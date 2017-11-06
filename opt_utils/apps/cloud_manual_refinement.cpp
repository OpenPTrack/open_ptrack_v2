/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016-, Matteo Munaro [matteo.munaro@dei.unipd.it]
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
 * cloud_refinement.cpp
 * Created on: Mar 24, 2016
 * Author: Matteo Munaro
 *
 */

// ROS includes:
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <dynamic_reconfigure/server.h>
#include <opt_utils/ManualCalibrationConfig.h>

// PCL includes:
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
//#include <fstream>
//#include <string>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//// Refinement matrix:
//Eigen::Affine3d registration_matrix;
//bool registration_matrix_read = false;
//// Output cloud:
PointCloudT::Ptr refined_cloud(new PointCloudT);
//// Output publisher:
ros::Publisher refined_cloud_pub;
std::string camera_name;
//tf::TransformListener* tf_listener;

Eigen::Transform<double, 3, Eigen::Affine> affine;
Eigen::Affine3d affine_world;
bool stop = false;
double voxel_x, voxel_y, voxel_z;
std::string frame_id;
tf::TransformListener* tf_listener;
Eigen::Affine3d extrinsic_transform_matrix = Eigen::Affine3d::Identity();

void
cloud_cb (const PointCloudT::ConstPtr& callback_cloud)
{
  camera_name = frame_id = callback_cloud->header.frame_id;
  tf::StampedTransform extrinsic_transform;
  tf_listener->waitForTransform(frame_id, "world", ros::Time(0), ros::Duration(0.5));
  tf_listener->lookupTransform(frame_id, "world", ros::Time(0), extrinsic_transform);
  tf::transformTFToEigen(extrinsic_transform, extrinsic_transform_matrix);

  // Create the filtering object
  PointCloudT cloud_filtered;
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (callback_cloud);
  sor.setLeafSize (voxel_x, voxel_y, voxel_z);
  sor.filter (cloud_filtered);

  pcl::transformPointCloud(cloud_filtered, *refined_cloud,
                           extrinsic_transform_matrix *
                           affine *
                           extrinsic_transform_matrix.inverse());
//  pcl::transformPointCloud(cloud_filtered, *refined_cloud, affine);
////  transform back to the correct frame_id
//  pcl::transformPointCloud(*refined_cloud, *refined_cloud,
//                           extrinsic_transform_matrix);

  refined_cloud_pub.publish(*refined_cloud);
}

void
save_cb(const std_msgs::EmptyConstPtr& e)
{
  stop = true;
}
void
save2_cb(const std_msgs::EmptyConstPtr& e)
{
  stop = true;
}

void
saveRegistrationMatrix (std::string filename, Eigen::Matrix4d transformation)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());
  myfile << transformation;
  myfile.close();
}

void configCallback(opt_utils::ManualCalibrationConfig &config,
                    uint32_t level)
{
  affine =
      Eigen::Translation3d(config.trs_x, config.trs_y, config.trs_z)
      * Eigen::AngleAxisd(config.rot_x, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(config.rot_y, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(config.rot_z, Eigen::Vector3d::UnitZ());
  if(config.voxel_z + config.voxel_y + config.voxel_x != 0)
  {
    voxel_x = config.voxel_x;
    voxel_y = config.voxel_y;
    voxel_z = config.voxel_z;
  }
  // transform affine from world to camera frame
  ROS_INFO_STREAM(camera_name << affine.matrix());
}

int
main (int argc, char** argv)
{
  stop = false;

  ros::init(argc, argv, "cloud_refinement");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  std::string input_topic;
  nh.param("input_topic", input_topic, std::string("input"));
  std::string output_topic;
  nh.param("output_topic", output_topic, std::string("output"));
  // Main loop rate:
  double rate_value;
  nh.param("rate", rate_value, 30.0);

  tf_listener = new tf::TransformListener();

  // Subscribers:
  ros::Subscriber sub = nh.subscribe<PointCloudT>(input_topic, 1, cloud_cb);
  ros::Subscriber sub2 = nh.subscribe<std_msgs::Empty>("save",1, save_cb);
  ros::Subscriber sub3 = nh.subscribe<std_msgs::Empty>("/save_all",1, save2_cb);

  // Publishers:
  refined_cloud_pub = nh.advertise<PointCloudT>(output_topic, 1);

  dynamic_reconfigure::Server<opt_utils::ManualCalibrationConfig> server;
  dynamic_reconfigure::Server<opt_utils::ManualCalibrationConfig>::CallbackType
      f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

  ros::Rate rate(rate_value);
  while(ros::ok() and not stop)
  {
    ros::spinOnce();

    rate.sleep();
  }

//  if(stop)
//  {
//    ROS_INFO_STREAM("Saving");
//    //save file
//    if (strcmp(camera_name.substr(0,1).c_str(), "/") == 0)  // Remove bar at the beginning
//    {
//      camera_name = camera_name.substr(1, camera_name.size() - 1);
//    }
//    std::string refinement_filename = ros::package::getPath("opt_calibration") + "/conf/registration_" + camera_name + ".txt";
//    if( boost::filesystem::exists(refinement_filename))
//    {
//      boost::filesystem::rename(refinement_filename, refinement_filename + "_old");
//    }
//    saveRegistrationMatrix(refinement_filename, affine_world.matrix());
//  }
  ros::shutdown();
  return 0;
}



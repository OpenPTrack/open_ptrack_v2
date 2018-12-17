/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <fstream>
#include <math.h>
#include "std_msgs/String.h"
#include <iostream>

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <opt_msgs/BorderPoint3D.h>
#include <opt_msgs/Border.h>
#include <opt_msgs/BorderArray.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/surface/concave_hull.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr concaveHull( pcl::PointCloud<pcl::PointXYZRGB>::Ptr );
tf::Vector3 measureBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
bool firstDetectionPlane( std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator );
bool findBoundingBox(std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator );
void sendBorderToRos();
void pp_callback ( const pcl::visualization::PointPickingEvent&, void* );
void sendClusterToUnity();
void showClusterCloud();
void listenerObj( const std_msgs::String::ConstPtr& );
void clusterExtraction( pcl::PointCloud<pcl::PointXYZ>::Ptr );


ros::Publisher publisher;

std::map<int, pcl::PointCloud<pcl::PointXYZ> > clusterCloud;
std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusterToSend;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > borders;

std::string path_obj_file;
std::string path_pcd_output;

int count = 0;
int min_cluster_points;
int max_cluster_points;

double rotation_plane_x;
double rotation_plane_y;
double rotation_plane_z;
double size_plane_x;
double size_plane_y;
double tolerance;

class variableCallback
{
  public:
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_point;
    variableCallback(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr points)
    {
      viewerPtr = viewer;
      clicked_point = points;
    }
};

template <typename T> std::string to_string(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

template <typename T> void saveCloud (const std::string &filename, const pcl::PointCloud<T> &cloud)
{
  pcl::PCDWriter w;
  w.writeBinaryCompressed (filename, cloud);
}

void sendBorderToRos()
{
  opt_msgs::BorderArray msgToSend;

  for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = clusterToSend.begin(); iter != clusterToSend.end(); ++iter)
  {
    opt_msgs::Border msg_border;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shapeBorder = concaveHull(iter->second);
    for(size_t i = 0; i < shapeBorder->points.size(); i++)
    {
      opt_msgs::BorderPoint3D pointSingle;
      pointSingle.x = shapeBorder->points[i].x;
      pointSingle.y = shapeBorder->points[i].y;
      pointSingle.z = shapeBorder->points[i].z;

      msg_border.border.push_back(pointSingle);
    }

    tf::Vector3 center;
    for(size_t i = 0; i< iter->second->points.size(); i++)
    {
      center[0] += iter->second->points[i].x;
      center[1] += iter->second->points[i].y;
      center[2] += iter->second->points[i].z;
    }

    center[0] /= iter->second->points.size();
    center[1] /= iter->second->points.size();
    center[2] /= iter->second->points.size();

    
    msg_border.center_x = center[0];
    msg_border.center_y = center[1];
    msg_border.center_z = center[2];

    tf::Vector3 boxBorder = measureBoundingBox(iter->second);
    
    msg_border.height_bbox = boxBorder[0];
    msg_border.width_bbox = boxBorder[1];
    msg_border.depth_bbox = boxBorder[2];

    msgToSend.borders.push_back(msg_border);
  }

  while(true)
  {
    publisher.publish(msgToSend);
    sleep(3);
    std::cout << "Message to ROS sent at time " << ros::Time::now() << std::endl;
    std::cout << "CTRL + C for quit" << std::endl << std::endl;
  }
}

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  if (event.getPointIndex () == -1)
    return;

  pcl::PointXYZRGB current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);

  variableCallback call = *((variableCallback*)args);
  pcl::visualization::PCLVisualizer::Ptr viewerPtr = call.viewerPtr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d = call.clicked_point;
  clicked_points_3d->points.push_back(current_point);

  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (clicked_points_3d, 255,0,0);
  
  std::stringstream sprint;
  sprint << "Point selected: " << clicked_points_3d->points.size();
  viewerPtr->updateText(sprint.str(), 10, 30, 255, 49, 255, "v2");

  viewerPtr->removePointCloud("clicked_points");
  viewerPtr->addPointCloud<pcl::PointXYZRGB>(clicked_points_3d, red, "clicked_points");
  viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  
  // std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}


void sendClusterToUnity()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d (new pcl::PointCloud<pcl::PointXYZRGB>);

  do{
    clicked_points_3d->points.clear(); 

    int c_j = 0;
    for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = clusterToSend.begin(); iter != clusterToSend.end(); ++iter)
    {
      if(c_j != iter->first)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr t_cloud = iter->second;
        clusterToSend.erase(iter->first);
        clusterToSend.insert(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::value_type(c_j, t_cloud));
      }
      c_j++;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer_fused ( new pcl::visualization::PCLVisualizer("Cloud Viewer fused"));
    viewer_fused->setCameraPosition(7,2,0, -5,0,15,  0,0,0,  0);

    viewer_fused->addText("!WARNING! The plane will be removed only if there is a RED point on it.", 10, 17, "v1", 0);

    std::stringstream sprint;
    sprint << "Point selected: " << clicked_points_3d->points.size();
    viewer_fused->addText(sprint.str(), 10, 30, 255, 49, 255, "v2", 0);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused (new pcl::PointCloud<pcl::PointXYZRGB>);

    for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = clusterToSend.begin(); iter != clusterToSend.end(); ++iter)
    {
      *fused += *iter->second;
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fused);
    viewer_fused->addPointCloud<pcl::PointXYZRGB> (fused, rgb, "fused cloud");

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clicked_points_3d (new pcl::PointCloud<pcl::PointXYZRGB>);

    variableCallback call(viewer_fused, clicked_points_3d);
    viewer_fused->registerPointPickingCallback (pp_callback, &call);

    std::cout << std::endl << "Select the plane that you want remove by SHIFT + CLICK" << std::endl;
    std::cout << "Press 'q' when you have done" << std::endl;

    // Spin until 'Q' is pressed:
    viewer_fused->spin();
    viewer_fused->setSize(1,1);
    viewer_fused->spinOnce();
    // viewer_fused->close();
    std::vector<int> pointInPlanes(clusterToSend.size());

    for(size_t pointSel = 0; pointSel < clicked_points_3d->points.size(); pointSel++)
    { 
      for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter = clusterToSend.begin(); iter != clusterToSend.end(); ++iter)
      { 
        for (size_t i = 0; i < iter->second->points.size(); i++)
        {
          if(clicked_points_3d->points[pointSel].x == iter->second->points[i].x &&
            clicked_points_3d->points[pointSel].y == iter->second->points[i].y &&
            clicked_points_3d->points[pointSel].z == iter->second->points[i].z)
              pointInPlanes[iter->first]++;
        }
      }
    }

    for(size_t j = 0; j < pointInPlanes.size(); j++)
    {
      if(pointInPlanes[j] % 2 == 1)
      {
        clusterToSend.erase(j);
        PCL_ERROR( "Plane %d removed by user\n", j );
      }
    }
  }while(clicked_points_3d->points.size() != 0);

  std::cout << std::endl << "Building message for ROS sender ..." << std::endl << std::endl;

  sendBorderToRos();
}

//Compute the border of the cloud received
pcl::PointCloud<pcl::PointXYZRGB>::Ptr concaveHull( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  
  copyPointCloud(*cloud_input, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  // std::cerr << "PointCloud after segmentation has: " << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  // std::cerr << "PointCloud after projection has: " << cloud_projected->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Concave hull has: " << cloud_hull->points.size()<< " data points." << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_temp->points.resize(cloud_hull->points.size());

  for (size_t i = 0; i < cloud_hull->points.size(); i++) {
    cloud_temp->points[i].x = cloud_hull->points[i].x;
    cloud_temp->points[i].y = cloud_hull->points[i].y;
    cloud_temp->points[i].z = cloud_hull->points[i].z;

    cloud_temp->points[i].r = 255;
    cloud_temp->points[i].g = 0;
    cloud_temp->points[i].b = 0;
  }
  // pcl::visualization::PCLVisualizer::Ptr viewerConcave ( new pcl::visualization::PCLVisualizer("Cloud Viewer concave"));
  // viewerConcave->setCameraPosition(7,2,0, -5,0,15,  0,0,0,  0);
  // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_temp);
  // viewerConcave->addPointCloud(cloud_temp, rgb, "concave");

  // while (!viewerConcave->wasStopped ())
  // {
  //   viewerConcave->spinOnce();
  // }

  return cloud_temp;
}

bool firstDetectionPlane(std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator iter)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

  copyPointCloud(iter->second, *cloud);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.001);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return false;
  }

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " " 
  //                                     << coefficients->values[3] << std::endl;

  Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector;

  floor_plane_normal_vector[0] = coefficients->values[0];
  floor_plane_normal_vector[1] = coefficients->values[1];
  floor_plane_normal_vector[2] = coefficients->values[2];

  if(std::abs(floor_plane_normal_vector[0]) > rotation_plane_x)
  {
    PCL_ERROR( "Plane deleted for rotation value on x = %f ( rad )\n", floor_plane_normal_vector[0] );
    return false;
  }
  std::cout << "Rotation value on x = " << floor_plane_normal_vector[0] << " ( rad ) "<< std::endl;

  if(std::abs(floor_plane_normal_vector[1]) > rotation_plane_y)
  {
    PCL_ERROR( "Plane deleted for rotation value on y = %f ( rad )\n ", floor_plane_normal_vector[1] );
    return false;
  }
  std::cout << "Rotation value on y = " << floor_plane_normal_vector[1] << " ( rad ) "<< std::endl;

  // if(std::abs(floor_plane_normal_vector[2]) > rotation_plane_z)
  // {
  //   PCL_ERROR( "Plane deleted for rotation value on z = %f ( rad )\n ", floor_plane_normal_vector[2] );
  //   return false;
  // }
  std::cout << "Rotation value on z = " << floor_plane_normal_vector[2] << " ( rad ) "<< std::endl;

  return true;
}

tf::Vector3 measureBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  copyPointCloud(*cloud_input, *cloud);

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);

  
  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));


  // Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  // Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  // Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  // Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

  // p1 = rotational_matrix_OBB * p1 + position;
  // p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  // p6 = rotational_matrix_OBB * p6 + position;
  // p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  float height = pow(pow((p8(0) - p4(0)), 2) + pow((p8(1) - p4(1)), 2) + pow((p8(2) - p4(2)), 2), 0.5);
  float width = pow(pow((p8(0) - p5(0)), 2) + pow((p8(1) - p5(1)), 2) + pow((p8(2) - p5(2)), 2), 0.5);
  float depth = pow(pow((p4(0) - p3(0)), 2) + pow((p4(1) - p3(1)), 2) + pow((p4(2) - p3(2)), 2), 0.5);

  return tf::Vector3(height, width, depth);
}


bool findBoundingBox(std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator iter)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  copyPointCloud(iter->second, *cloud);

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);

  
  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));


  // Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  // Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  // Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
  Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
  // Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  // Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
  Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

  // p1 = rotational_matrix_OBB * p1 + position;
  // p2 = rotational_matrix_OBB * p2 + position;
  // p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  // p6 = rotational_matrix_OBB * p6 + position;
  // p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  float height = pow(pow((p8(0) - p4(0)), 2) + pow((p8(1) - p4(1)), 2) + pow((p8(2) - p4(2)), 2), 0.5);
  float width = pow(pow((p8(0) - p5(0)), 2) + pow((p8(1) - p5(1)), 2) + pow((p8(2) - p5(2)), 2), 0.5);

  if( width > size_plane_x )
  {
    if( height > size_plane_y )
    {
      std::cerr << "Height ( 8, 4 ) : " << height << std::endl;
      std::cerr << "Width  ( 8, 5 ) : " << width << std::endl;

      // viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB_" + to_string(iter->first));
      // viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector"  + to_string(iter->first));
      // viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector" + to_string(iter->first));
      // viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector" + to_string(iter->first));

      return true;
    }
    else
    {
      PCL_ERROR( "Plane deleted for height value = %f ( m )\n ", height );
    }
  }
  else
  {
    PCL_ERROR( "Plane deleted for width value = %f ( m )\n ", width );
  }

    return false;

}

void showClusterCloud()
{  
  for(std::map<int, pcl::PointCloud<pcl::PointXYZ> >::iterator iter = clusterCloud.begin(); iter != clusterCloud.end(); ++iter)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud = iter->second;
    
    std::cerr << std::endl << "---> Cluster #" << iter->first << std::endl;
    std::cerr << "Size: " << cloud.height * cloud.width << " data points." << std::endl;
    
    int red = int(rand()%256);
    int green = int(rand()%256);
    int blue = int(rand()%256);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    cloud_temp->points.resize(cloud.height * cloud.width);

    for (size_t i = 0; i < cloud.points.size(); i++) {
      cloud_temp->points[i].x = cloud.points[i].x;
      cloud_temp->points[i].y = cloud.points[i].y;
      cloud_temp->points[i].z = cloud.points[i].z;

      cloud_temp->points[i].r = red;
      cloud_temp->points[i].g = green;
      cloud_temp->points[i].b = blue;
    }

    if( firstDetectionPlane(iter) )
    {
      std::cout << "Plane with NORMAL approvated!" << std::endl;

      if( findBoundingBox(iter) )
      {
        std::cout << "Plane with SIZE approvated!" << std::endl;

        clusterToSend.insert(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::value_type(iter->first, cloud_temp));

        std::cout << "Plane added!" << std::endl;
      }
    }
  }

  sendClusterToUnity();
}

void clusterExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  do{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_points);
    ec.setMaxClusterSize (max_cluster_points);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
      
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusterCloud.insert(std::map<int, pcl::PointCloud<pcl::PointXYZ> >::value_type(clusterCloud.size(), *cloud_cluster));
    }

    //Modify the parameter number for a possible re-computation
    min_cluster_points *= 0.8;
    // tolerance -= 0.01;

    std::cout << "Cluster founded: " << clusterCloud.size() << std::endl;

  }while(clusterCloud.size() == 0);
  
  showClusterCloud(); 

}

void objToPclVisualizer ()
{
  std::cout << "Convert a OBJ file to PCD format: " << path_obj_file.c_str() << std::endl;

  // Load the input file
  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New ();
  reader->SetFileName (path_obj_file.c_str());
  reader->Update ();
  polydata = reader->GetOutput ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::io::vtkPolyDataToPointCloud (polydata, *cloud);
  // Convert to pcd and save
  saveCloud (path_pcd_output.c_str(), *cloud);

  pcl::visualization::PCLVisualizer::Ptr viewerOriginal ( new pcl::visualization::PCLVisualizer("Cloud Viewer original"));
  viewerOriginal->setCameraPosition(7,2,0, -5,0,15,  0,0,0,  0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud_temp->points.resize(cloud->height * cloud->width);

  for (size_t i = 0; i < cloud->points.size(); i++) {
    cloud_temp->points[i].x = cloud->points[i].x;
    cloud_temp->points[i].y = cloud->points[i].y;
    cloud_temp->points[i].z = cloud->points[i].z;

    cloud_temp->points[i].r = 255;
    cloud_temp->points[i].g = 255;
    cloud_temp->points[i].b = 255;
  }

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_temp);
  viewerOriginal->addPointCloud(cloud_temp, rgb, "original");

  clusterExtraction(cloud);
}

void listenerObj(const std_msgs::String::ConstPtr& msg)
{
  try
  {
    if(count == 0)
    {
      std::cout << "Receiving file ... " << std::endl;
    }

    std::string line = msg->data.c_str();
    if(line.compare("-!-!-!-!-") == 0)
    {  
      std::cout << "Line arrived # " << count << std::endl;
      std::cout << "Finish file -> Write file in: " << path_obj_file.c_str() << std::endl;
      count = 0;

      std::cout << "\n\nPlane detection started ...\n" << std::endl;

      objToPclVisualizer();
      
      exit(0);
    }

    std::ofstream fileObj (path_obj_file.c_str(), std::ofstream::app);
    fileObj << line << std::endl;
    fileObj.close();

    count++;
  }
  catch (std::exception e)
  {
    PCL_ERROR("Error! No message in input");
    return;
  }

}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "Receiver");
  ros::NodeHandle nh;

  srand(time(0));   

  std::string topic_obj;
  nh.param("plane_detector/topic_obj", topic_obj, std::string("/mrroom/objFromUnity"));
  // ROS_WARN("Got param topic_obj: %s", topic_obj.c_str());

  std::string output_topic;
  nh.param("plane_detector/output_topic", output_topic, std::string("/mrroom/borders"));
  // ROS_WARN("Got param output_topic: %s", output_topic.c_str());

  nh.param("plane_detector/path_obj_file", path_obj_file, std::string(""));
  // ROS_WARN("Got param path_obj_input: %s", path_obj_file.c_str());

  nh.param("plane_detector/path_output", path_pcd_output, std::string(""));
  // ROS_WARN("Got param path_output: %s", path_pcd_output.c_str());


  //Reset parameter loading the original values
  nh.param("/plane_detector/rotation_plane_x", rotation_plane_x, 0.6);
  nh.param("/plane_detector/rotation_plane_y", rotation_plane_y, 0.6);
  // nh.param("/plane_detector/rotation_plane_z", rotation_plane_z, 0.0);
  nh.param("/plane_detector/size_plane_x", size_plane_x, 0.35);
  nh.param("/plane_detector/size_plane_y", size_plane_y, 0.35);
  nh.param("/plane_detector/min_cluster_points", min_cluster_points, 3000);
  nh.param("/plane_detector/max_cluster_points", max_cluster_points, 25000);
  nh.param("/plane_detector/tolerance", tolerance, 0.026);

  publisher = nh.advertise<opt_msgs::BorderArray>(output_topic.c_str(), 1000);

  std::ofstream fileObj (path_obj_file.c_str());
  fileObj.close();

  PCL_ERROR( "\n\nWaiting file ...\n\n");

  ros::Subscriber sub = nh.subscribe(topic_obj.c_str(), 1000, listenerObj);

  ros::spin();
   
  return (0);
}
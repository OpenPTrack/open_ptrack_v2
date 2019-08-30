// ROS includes:
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

// PCL includes:
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

// Open PTrack includes:
#include <open_ptrack/detection/ground_segmentation.h>
#include <open_ptrack/detection/ground_based_people_detection_app.h>
#include <open_ptrack/opt_utils/conversions.h>

//Publish Messages
#include <opt_msgs/RoiRect.h>
#include <opt_msgs/Rois.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>

// Dynamic reconfigure:
#include <dynamic_reconfigure/server.h>
#include <detection/GroundBasedPeopleDetectorConfig.h>

using namespace opt_msgs;
using namespace sensor_msgs;

ros::Publisher pub;

void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& pc2){

    sensor_msgs::PointCloud2 cloud(*pc2);

    static tf::TransformBroadcaster br;
    tf::Vector3 tran_input = tf::Vector3(0,0,0);
    tf::Quaternion quat_input = tf::Quaternion(0,1,0,0);

    tf::Transform transform;
    transform.setOrigin(tran_input);
    transform.setRotation(quat_input);

    cloud.header.frame_id = "new" + pc2->header.frame_id;
    br.sendTransform(tf::StampedTransform(transform, pc2->header.stamp, cloud.header.frame_id, pc2->header.frame_id));

   pub.publish(cloud);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "zed_rot");

  ros::NodeHandle n;

  pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud_rot", 1000);

  ros::Subscriber sub = n.subscribe("/zed/point_cloud/cloud_registered", 1000, pc2Callback);
  ros::spin();

  return 0;
}
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"


static ros::Publisher vis_pub;

void publishMarker(tf::Vector3 tran_input, tf::Quaternion quat_input)
{
  visualization_msgs::Marker marker_pose;
  marker_pose.header.frame_id = "kinect2_head_rgb_optical_frame";
  // marker_pose.header.stamp = header.stamp;
  marker_pose.ns = "mobile_phone";
  marker_pose.id = 0;
  marker_pose.type = visualization_msgs::Marker::SPHERE;
  marker_pose.action = visualization_msgs::Marker::ADD;
  marker_pose.pose.position.x = tran_input[0];
  marker_pose.pose.position.y = tran_input[1];
  marker_pose.pose.position.z = tran_input[2];
  marker_pose.pose.orientation.x = quat_input[0];
  marker_pose.pose.orientation.y = quat_input[1];
  marker_pose.pose.orientation.z = quat_input[2];
  marker_pose.pose.orientation.w = quat_input[3];
  marker_pose.scale.x = 0.05;
  marker_pose.scale.y = 0.05;
  marker_pose.scale.z = 0.05;
  marker_pose.color.a = 1.0;
  marker_pose.color.r = 0.33;
  marker_pose.color.g = 0.94;
  marker_pose.color.b = 0.20;
  marker_pose.lifetime = ros::Duration(5.0);

  vis_pub.publish(marker_pose);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "arcore_tracker");
  ros::NodeHandle nh_trk;

  vis_pub = nh_trk.advertise<visualization_msgs::Marker>("/arcore/marker_array", 0);


  tf::TransformListener listener;

  while(ros::ok())
  {
    tf::StampedTransform transform;
    
    try{
      listener.lookupTransform("/arcore", "/kinect2_head_rgb_optical_frame", ros::Time(0), transform);  
    }
    catch(tf::TransformException &ex){
      // ROS_ERROR("%s", ex.what());
      continue;
    }

    std::cout << transform.getOrigin().x() << std::endl;
    publishMarker(transform.getOrigin(), transform.getRotation());

    ros::spinOnce();
  }

  ros::spin();

  return 0;
}

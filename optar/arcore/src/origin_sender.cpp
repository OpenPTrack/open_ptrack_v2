/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "Origin_sender");
  ros::NodeHandle nh_origin;

  tf::TransformListener listener;      
  tf::StampedTransform transform;

  sleep(3.0);

  ros::Rate loop_rate_error(1);
  ros::Rate loop_rate(30);
  bool countClass = false;


  std::string nameTag;
  nh_origin.param("origin_sender/name_tag", nameTag, std::string("tag_0_arcore"));
  ROS_WARN("ORIGIN -> Got param name_tag: %s", nameTag.c_str());

  std::string output_topic;
  nh_origin.param("origin_sender/output_topic", output_topic, std::string("arcore/origin"));
  ROS_WARN("ORIGIN -> Got param output_topic: %s", output_topic.c_str());

  ros::Publisher vis_pub = nh_origin.advertise<geometry_msgs::PoseStamped>(output_topic.c_str(), 100);

  while(ros::ok())
  {
    try{      
      listener.lookupTransform(nameTag.c_str(), "world", ros::Time(0), transform);

      geometry_msgs::PoseStamped message; 
      message.header.frame_id = nameTag.c_str();
      message.header.stamp = ros::Time::now();
      message.pose.position.x = transform.getOrigin().x();
      message.pose.position.y = transform.getOrigin().y();
      message.pose.position.z = transform.getOrigin().z();
      message.pose.orientation.x = transform.getRotation().x();
      message.pose.orientation.y = transform.getRotation().y();
      message.pose.orientation.z = transform.getRotation().z();
      message.pose.orientation.w = transform.getRotation().w();
      vis_pub.publish(message);

      if(!countClass)
      {
        ROS_INFO("ORIGIN -> Write transformation: %s -> %s", nameTag.c_str(), "world");
        countClass = true;
      }


    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("ORIGIN -> %s", ex.what());
      loop_rate_error.sleep();
      continue;
    }
    loop_rate.sleep();
    ros::spinOnce();
  }


  return 0;
}

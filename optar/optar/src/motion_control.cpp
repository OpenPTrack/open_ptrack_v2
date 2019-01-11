/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/
#include <iostream>
#include <iomanip>
#include <ctime>

#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

std::string filePathPhone;
std::string filePathTag;
std::string nameTag;
tf::TransformListener* listener;

int count = 0;

void listenerTag(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  try
  {
    if(msg->poses.size() == 0)
    {
      ROS_WARN("MOTION CONTROL -> No tag detected. Waiting tag detection...");
      return;
    }

    static tf::TransformBroadcaster br;

    tf::Transform transformToSend;
    transformToSend.setOrigin(tf::Vector3(0,0.38,0));
    transformToSend.setRotation(tf::Quaternion(0,0,0,1));

    br.sendTransform(tf::StampedTransform(transformToSend, ros::Time::now(), nameTag.c_str(), "diffTag"));


    // frameParent = msg->header.frame_id.c_str();
    //ROS_INFO("FILE TAG -> Detected tag with parent: %s", frameParent.c_str());

    tf::StampedTransform transformTag;
    tf::StampedTransform transformPhone;

    listener->lookupTransform("world", "diffTag", ros::Time(0), transformTag);  
    listener->lookupTransform("world", "phone", ros::Time(0), transformPhone);  

    double distance = hypot(hypot(transformTag.getOrigin().x() - transformPhone.getOrigin().x(), transformTag.getOrigin().y() - transformPhone.getOrigin().y()), transformTag.getOrigin().z() - transformPhone.getOrigin().z());
    // ROS_INFO("MOTION CONTROL -> Distance %f", distance);

    if(distance > 0.4)
    {
        ROS_INFO("MOTION CONTROL -> Distance problem -> No data printed. d = %f", distance);
        return;
    }

    std::ofstream fileTag (filePathTag.c_str(), std::ofstream::app);

    fileTag << transformTag.getOrigin().x() << " ";
    fileTag << transformTag.getOrigin().y() << " ";
    fileTag << transformTag.getOrigin().z() << std::endl;
    // file << transformTag.getRotation().x() << std::endl;
    // file << transformTag.getRotation().y() << std::endl;
    // file << transformTag.getRotation().z() << std::endl;
    // file << transformTag.getRotation().w() << std::endl;

    fileTag.close();

    std::ofstream filePhone (filePathPhone.c_str(), std::ofstream::app);

    filePhone << transformPhone.getOrigin().x() << " ";
    filePhone << transformPhone.getOrigin().y() << " ";
    filePhone << transformPhone.getOrigin().z() << std::endl;
    // file << transformPhone.getRotation().x() << std::endl;
    // file << transformPhone.getRotation().y() << std::endl;
    // file << transformPhone.getRotation().z() << std::endl;
    // file << transformPhone.getRotation().w() << std::endl;
    
    filePhone.close();

    ROS_INFO("-----------------");
    ROS_INFO("-----------------");
    ROS_INFO("-----------------");
    ROS_INFO("-----------------");
    ROS_INFO("-----------------");

    count++;
    ROS_WARN("MOTION CONTROL -> File write with seq# %d ______ record # %d", msg->header.seq, count);

  }
  catch (tf::TransformException e)
  {
    ROS_ERROR("MOTION CONTROL -> Error! Lookup transform: %s", e.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Motion_control");
  ros::NodeHandle nh_tag;

  sleep(3.0);

  tf::TransformListener listener2;
  listener = &listener2;

  nh_tag.param("motion_control/file_path_tag", filePathTag, std::string(""));
  ROS_WARN("MOTION CONTROL -> Got param file_path_tag: %s", filePathTag.c_str());

  nh_tag.param("motion_control/file_path_phone", filePathPhone, std::string(""));
  ROS_WARN("MOTION CONTROL -> Got param file_path_phone: %s", filePathPhone.c_str());

  std::string input_topic;
  nh_tag.param("motion_control/input_topic", input_topic, std::string("/camera_network/tag_detections_pose"));
  ROS_WARN("MOTION CONTROL -> Got param frame_parent: %s", input_topic.c_str());

  nh_tag.param("motion_control/name_tag", nameTag, std::string("/tag_1"));
  ROS_WARN("MOTION CONTROL -> Got param tag_name: %s", nameTag.c_str());
  
  //Initialize Tag file
  std::ofstream fileTag (filePathTag.c_str());
  fileTag << "World -> " <<  nameTag.c_str() << std::endl;
  fileTag.close();

  //Initialize Phone file
  std::ofstream filePhone (filePathPhone.c_str());
  filePhone << "World -> Phone" << std::endl;
  filePhone.close();
  
  ros::Subscriber sub = nh_tag.subscribe(input_topic.c_str(), 1000, listenerTag);

  ros::spin();
  return 0;
}

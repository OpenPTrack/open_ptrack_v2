/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"

std::string filePath;
std::string nameTag;
tf::TransformListener* listener;
bool countClass = false;

void writeToFileTopic(std::string frameParent)
{
  try{

    tf::StampedTransform transform;
    
    listener->lookupTransform(frameParent.c_str(), nameTag.c_str(), ros::Time(0), transform);  
    
    std::ofstream file (filePath.c_str());

    file << frameParent.c_str() << std::endl;
    file << nameTag.c_str() << std::endl;

    file << transform.getOrigin().x() << std::endl;
    file << transform.getOrigin().y() << std::endl;
    file << transform.getOrigin().z() << std::endl;
    file << transform.getRotation().x() << std::endl;
    file << transform.getRotation().y() << std::endl;
    file << transform.getRotation().z() << std::endl;
    file << transform.getRotation().w() << std::endl;

    file.close();


    if(!countClass)
    {
      ROS_INFO("FILE TAG -> Write transformation: %s -> %s", frameParent.c_str(), nameTag.c_str());
      countClass = true;
    }
    
    // ROS_INFO("FILE TAG -> Write transformation: %s -> %s", frameParent.c_str(), nameTag.c_str());

  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("FILE TAG -> %s", ex.what());
    std::ofstream file (filePath.c_str());
    file << "no data" << std::endl;
    file.close();

    return;
  }
}

void listenerTag(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  try
  {
    if(msg->poses.size() == 0 && countClass == 0)
    {
      ROS_WARN("FILE TAG -> No tag detected. Waiting first tag detection...");
      return;
    }
    // frameParent = msg->header.frame_id.c_str();
    //ROS_INFO("FILE TAG -> Detected tag with parent: %s", frameParent.c_str());

    writeToFileTopic(msg->header.frame_id);
  }
  catch (std::exception e)
  {
    ROS_ERROR("FILE TAG -> Error! No message in input");
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Tag_file_node");
  ros::NodeHandle nh_tag;

  sleep(3.0);

  tf::TransformListener listener2;
  listener = &listener2;

  nh_tag.param("tag_file/file_path", filePath, std::string(""));
  ROS_WARN("FILE TAG -> Got param file_path: %s", filePath.c_str());

  std::string input_topic;
  nh_tag.param("tag_file/input_topic", input_topic, std::string("/camera_network/tag_detections_pose"));
  ROS_WARN("FILE TAG -> Got param frame_parent: %s", input_topic.c_str());

  nh_tag.param("tag_file/name_tag", nameTag, std::string("/tag_0"));
  ROS_WARN("FILE TAG -> Got param tag_name: %s", nameTag.c_str());
  
  ros::Subscriber sub = nh_tag.subscribe(input_topic.c_str(), 1000, listenerTag);

  ros::spin();
  return 0;
}

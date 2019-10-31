/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"

int countClass = 0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Keep_tag_living_node");
  ros::NodeHandle nh_tag;
  
  sleep(3.0);

  ros::Rate loopRate(1);

  std::string output_name_tag;
  nh_tag.param("keep_tag_living/name_tag", output_name_tag, std::string("/tag_0_arcore"));
  ROS_WARN("TAG -> Got param name_tag: %s", output_name_tag.c_str());

  std::string filePath;
  nh_tag.param("keep_tag_living/file_path", filePath, std::string(""));
  ROS_WARN("TAG -> Got param file_path: %s", filePath.c_str());
  
  static tf::TransformBroadcaster br;

  std::string noData = "no data";

  //Erase data from the previous detection
  
  std::ofstream file (filePath.c_str());
  file << "no data" << std::endl;
  file.close();

  while(ros::ok())
  {
      try{
      std::ifstream inputFile(filePath.c_str());
      std::string line;

      float arrayInput[7];
      int count = 0;

      std::string parentFrame;
      std::string childFrame;

      while(std::getline(inputFile, line))
      {
        if(noData.compare(line) == 0 && countClass == 0)
        {
          ROS_ERROR("TAG -> NO DATA in %s", filePath.c_str());
          loopRate.sleep();
          break;
        }

        if(count == 0)
          parentFrame = line;
        else if(count == 1)
          childFrame = line;
        else
        {
          float number;
          std::stringstream ss(line);
          ss >> number;
          arrayInput[count-2] = number;
        }
        count++;
      }

      if(count == 9)
      {
        inputFile.close();
        tf::StampedTransform transform;
        transform.setOrigin( tf::Vector3(arrayInput[0], arrayInput[1], arrayInput[2]) );
        transform.setRotation( tf::Quaternion( arrayInput[3], arrayInput[4], arrayInput[5], arrayInput[6]) );
      
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parentFrame, output_name_tag.c_str()));

        // ROS_INFO("TAG -> Published tag pose from file");

        // ROS_INFO("TAG -> Parent: %s", parentFrame.c_str());
        // ROS_INFO("TAG -> Child: %s", childFrame.c_str());

        // ROS_INFO("TAG -> Trans: %f * %f * %f", arrayInput[0], arrayInput[1], arrayInput[2]);
        // ROS_INFO("TAG -> Rot: %f * %f * %f * %f", arrayInput[3], arrayInput[4], arrayInput[5], arrayInput[6]);

        if(countClass == 0)
        {
          ROS_INFO("TAG -> Published tag pose from file");
          countClass++;
        }
      }      

    }
    catch(std::exception &ex)
    {
      ROS_ERROR("TAG -> %s", ex.what());
    }

    ros::spinOnce();
  }

  return 0;
}

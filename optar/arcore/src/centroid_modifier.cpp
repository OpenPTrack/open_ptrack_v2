/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/Track.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"


tf::Vector3 originTrans;
tf::Quaternion originQuat;
ros::Publisher pub;
bool count = false;

void listenerVodom(const opt_msgs::TrackArray::ConstPtr& msg)
{
  opt_msgs::TrackArray msgMod(*msg);
  try
  {    

	tf::Matrix3x3 matrixQuat(tf::Quaternion(originQuat.getX(), originQuat.getY(), originQuat.getZ(), originQuat.getW()));
  	tf::Vector3 transOrig2(originTrans.getX(), originTrans.getY(), originTrans.getZ());
	
  	tf::Transform transform(matrixQuat, transOrig2);

	for(int i = 0; i < msg->tracks.size(); i++)
	{
		

		tf::Matrix3x3 matrixElement(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
		tf::Vector3 translationElement(msg->tracks[i].x, msg->tracks[i].y, msg->tracks[i].height);
		
		tf::Transform transformElement(matrixElement, translationElement);
		
		tf::Transform multi = transform * transformElement;
		
		//static tf::TransformBroadcaster br;
		//std::string print = "c__" + msg->tracks[i].id;
		//br.sendTransform(tf::StampedTransform(multi, ros::Time::now(), "tag_0_arcore", print));
		
		msgMod.tracks[i].x = multi.getOrigin().x();
		msgMod.tracks[i].y = multi.getOrigin().y();
		msgMod.tracks[i].height = multi.getOrigin().z();
		
		
	}

	pub.publish(msgMod);
	if(!count)
	{
		ROS_INFO("MODIFIER -> Started");
		count = true;
	}

  }
  catch (std::exception e)
  {
      ROS_ERROR("MODIFIER -> No message from detector");
      sleep(3.0);
      return;
  }
}

void listenerOrigin(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  try
  {    
	
	originTrans = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	originQuat = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  }
  catch (std::exception e)
  {
      ROS_ERROR("MODIFIER -> No message from origin");
      sleep(3.0);
      return;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "modifier");
  ros::NodeHandle nh;

  sleep(3.0);

  std::string origin;
  nh.param("centroid_modifier/origin", origin, std::string("/arcore/origin"));
  ROS_WARN("MODIFIER -> Got param origin: %s", origin.c_str());
	
  std::string input_track;
  nh.param("centroid_modifier/input_track", input_track, std::string("/tracker/tracks_smoothed"));
  ROS_WARN("MODIFIER -> Got param input_track: %s", input_track.c_str());

  std::string output_topic;
  nh.param("centroid_modifier/output_topic", output_topic, std::string("arcore/centroid_mod"));
  ROS_WARN("MODIFIER -> Got param output_topic: %s", output_topic.c_str());

  pub = nh.advertise<opt_msgs::TrackArray>(output_topic.c_str(), 100);
  ros::Subscriber sub = nh.subscribe(input_track.c_str(), 1000, listenerVodom);
  ros::Subscriber sub2 = nh.subscribe(origin.c_str(), 1000, listenerOrigin);

  ros::spin();

  return 0;
}

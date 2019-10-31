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
#include "../utils.hpp"

/*
tf::Vector3 originTrans;
tf::Quaternion originQuat;
*/
ros::Publisher pub;
bool count = false;
std::string nameTag;
std::string output_topic;

std::shared_ptr<tf::TransformListener> tfListener;

void listenerOPTCentroid(const opt_msgs::TrackArray::ConstPtr& msg)
{
  opt_msgs::TrackArray msgMod(*msg);
  try
  {    


    //ROS_INFO("got centroids");
    //ROS_INFO_STREAM("tag name = "<<nameTag);

    tf::StampedTransform transform;
    bool gotTransform = false;
    std::string targetFrame = nameTag;
    std::string sourceFrame = "/world";
    try
    {
      tfListener->waitForTransform(targetFrame, sourceFrame, msg->header.stamp, ros::Duration(1));
      tfListener->lookupTransform(nameTag, sourceFrame, msg->header.stamp, transform);
      gotTransform = true;
    }
    catch (tf::LookupException e)
    {
      ROS_WARN_STREAM("Frame doesn't currently exist: what()="<<e.what());
    }
    catch(tf::ConnectivityException e)
    {
      ROS_ERROR_STREAM("Frames are not connected: what()="<<e.what());
    }
    catch(tf::ExtrapolationException e)
    {
      ROS_WARN_STREAM("Centroid timestamp is too far off from now: what()="<<e.what());
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR_STREAM(e.what());
    }
    if(!gotTransform)
      return;

    /*
	  tf::Matrix3x3 matrixQuat(tf::Quaternion(originQuat.getX(), originQuat.getY(), originQuat.getZ(), originQuat.getW()));
  	tf::Vector3 transOrig2(originTrans.getX(), originTrans.getY(), originTrans.getZ());	
  	tf::Transform transform(matrixQuat, transOrig2);

    ROS_INFO("from tf vs from msg:");
    ROS_INFO_STREAM("    "<<poseToString(tfTransform));
    ROS_INFO_STREAM("    "<<poseToString(transform));
    */

  	for(size_t i = 0; i < msg->tracks.size(); i++)
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
  		msgMod.tracks[i].height = multi.getOrigin().z() - 0.45;
  		
  		msgMod.header.frame_id = targetFrame;
  	}

  	pub.publish(msgMod);
    //ROS_INFO("Published transformed centroid data");

  	if(!count)
  	{
  		ROS_INFO_STREAM("Started publishing transformed centroids on topic "<<output_topic);
  		count = true;
  	}

  }
  catch (std::exception e)
  {
      ROS_ERROR("MODIFIER CEN -> No message from detector");
      sleep(3.0);
      return;
  }
}


/*
void listenerOrigin(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  try
  {    
	
	originTrans = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	originQuat = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

  }
  catch (std::exception e)
  {
      ROS_ERROR("MODIFIER CEN -> No message from origin");
      sleep(3.0);
      return;
  }
}

*/


int main(int argc, char **argv)
{
  ros::init(argc, argv, "modifier_cen");
  ros::NodeHandle nh;

  sleep(3.0);

  std::string origin;
  nh.param("centroid_modifier/origin", origin, std::string("/arcore/origin"));
  ROS_WARN("MODIFIER CEN -> Got param origin: %s", origin.c_str());
	
  std::string input_track;
  nh.param("centroid_modifier/input_track", input_track, std::string("/tracker/tracks_smoothed"));
  ROS_WARN("MODIFIER CEN -> Got param input_track: %s", input_track.c_str());

  nh.param("centroid_modifier/output_topic", output_topic, std::string("/arcore/tracks_smoothed"));
  ROS_WARN("MODIFIER CEN -> Got param output_topic: %s", output_topic.c_str());

  nh.param("origin_sender/name_tag", nameTag, std::string("tag_0_arcore"));
  ROS_WARN("ORIGIN -> Got param name_tag: %s", nameTag.c_str());

  pub = nh.advertise<opt_msgs::TrackArray>(output_topic.c_str(), 10);
  ros::Subscriber sub = nh.subscribe(input_track.c_str(), 1, listenerOPTCentroid);
  //ros::Subscriber sub2 = nh.subscribe(origin.c_str(), 1000, listenerOrigin);

  tfListener = std::make_shared<tf::TransformListener>(ros::Duration(60));//60 seconds buffer for old data
  ros::spin();

  return 0;
}

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
#include <opt_msgs/SkeletonTrack.h>
#include <opt_msgs/SkeletonTrackArray.h>
#include "std_msgs/String.h"
/*
tf::Vector3 originTrans;
tf::Quaternion originQuat;
*/
ros::Publisher pub;
bool countClass = false;
std::string nameTag;

std::shared_ptr<tf::TransformListener> tfListener;

void listenerOPTSkeleton(const opt_msgs::SkeletonTrackArrayConstPtr& msg )
{
  opt_msgs::SkeletonTrackArray msgMod(*msg);

  //Create a Transform with the tag -> world transform
  /*
  tf::Matrix3x3 matrixQuat(tf::Quaternion(originQuat.getX(), originQuat.getY(), originQuat.getZ(), originQuat.getW()));
	tf::Vector3 transOrig2(originTrans.getX(), originTrans.getY(), originTrans.getZ());
	tf::Transform transform(matrixQuat, transOrig2);
  */

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




  //Modify centroid and each joint for each person    
  for(size_t count = 0; count < msg->tracks.size(); count++)
  {
    opt_msgs::SkeletonTrack track = msg->tracks[count];
    // ROS_INFO("MODIFIER SKE -> %f, %f, %f", track.x, track.y, track.height);

    tf::Matrix3x3 matrixElement(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    //Transform centroid
    tf::Vector3 translationElement(track.x, track.y, 0.0);
    tf::Transform transformElement(matrixElement, translationElement);


    tf::Transform multi = transform * transformElement;
    
    //Set new values to new msg
    msgMod.tracks[count].x = multi.getOrigin().x();
    msgMod.tracks[count].y = multi.getOrigin().y();
    msgMod.tracks[count].height = multi.getOrigin().z();


    for(size_t counterJoint = 0; counterJoint < track.joints.size(); counterJoint++)
    {
      //Transform joint #counterJoint
      tf::Vector3 translationElementJoint(track.joints[counterJoint].x, track.joints[counterJoint].y, track.joints[counterJoint].z);
      tf::Transform transformElementJoint(matrixElement, translationElementJoint);

      tf::Transform multiJoint = transform * transformElementJoint;

      //Set new values to new msg
      msgMod.tracks[count].joints[counterJoint].x = multiJoint.getOrigin().x();
      msgMod.tracks[count].joints[counterJoint].y = multiJoint.getOrigin().y();
      msgMod.tracks[count].joints[counterJoint].z = multiJoint.getOrigin().z();
    }
  }

  //msgMod.header.frame_id = targetFrame;

  pub.publish(msgMod);

  ROS_INFO("MODIFIER SKE -> Published transform with seq numb: %d", msgMod.header.seq);

  if(!countClass)
  {
    ROS_INFO("MODIFIER SKE -> Started");
    countClass = true;
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
      ROS_ERROR("MODIFIER SKE -> No message from origin");
      sleep(3.0);
      return;
  }
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "modifier_ske");
  ros::NodeHandle nh;

  sleep(3.0);

  std::string origin;
  nh.param("skeleton_modifier/origin", origin, std::string("/arcore/origin"));
  ROS_WARN("MODIFIER SKE -> Got param origin: %s", origin.c_str());
	
  std::string input_track;
  nh.param("skeleton_modifier/input_track", input_track, std::string("/tracker/skeleton_tracks"));
  ROS_WARN("MODIFIER SKE -> Got param input_track: %s", input_track.c_str());

  std::string output_topic;
  nh.param("skeleton_modifier/output_topic", output_topic, std::string("/arcore/skeleton_tracks"));
  ROS_WARN("MODIFIER SKE -> Got param output_topic: %s", output_topic.c_str());

  nh.param("origin_sender/name_tag", nameTag, std::string("tag_0_arcore"));
  ROS_WARN("ORIGIN -> Got param name_tag: %s", nameTag.c_str());

  pub = nh.advertise<opt_msgs::SkeletonTrackArray>(output_topic.c_str(), 10);
  ros::Subscriber sub = nh.subscribe(input_track.c_str(), 1, listenerOPTSkeleton);
  //ros::Subscriber sub2 = nh.subscribe(origin.c_str(), 1, listenerOrigin);

  tfListener = std::make_shared<tf::TransformListener>(ros::Duration(60));//60 seconds buffer for old data

  ros::spin();

  return 0;
}

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

tf::StampedTransform transform;
ros::Publisher pub;
bool countClass = false;
bool firstTransf = false;
tf::TransformListener* listener;
std::string kinect;

void listenerOPTSkeleton(const opt_msgs::SkeletonTrackArrayConstPtr& msg )
{
  opt_msgs::SkeletonTrackArray msgMod(*msg);
  
  try{
      listener->lookupTransform(kinect.c_str(), "/world", ros::Time(0), transform);  

      // originTrans = tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      // originQuat = tf::Quaternion(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());

      // ROS_INFO("%f __ %f __ %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
      firstTransf = true;
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("SKE -> %s", ex.what());
  }

  if(!firstTransf)
    return;

  try
  {    
    //Create a Transform with the tag -> world transform
	  tf::Matrix3x3 matrixQuat(tf::Quaternion(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w()));
  	tf::Vector3 transOrig2(tf::Vector3(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z()));
  	tf::Transform transform(matrixQuat, transOrig2);

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

        if(counterJoint == 4)
        {
          static tf::TransformBroadcaster br;
          tf::Transform transformToSend;
          transformToSend.setOrigin(tf::Vector3(multiJoint.getOrigin().x(), multiJoint.getOrigin().y(), multiJoint.getOrigin().z()));
          transformToSend.setRotation(tf::Quaternion(0,0,0,1));
          br.sendTransform(tf::StampedTransform(transformToSend, ros::Time::now(), kinect.c_str(), "hand"));
        }
        
        //Set new values to new msg
        msgMod.tracks[count].joints[counterJoint].x = multiJoint.getOrigin().x();
        msgMod.tracks[count].joints[counterJoint].y = multiJoint.getOrigin().y();
        msgMod.tracks[count].joints[counterJoint].z = multiJoint.getOrigin().z();
      }
    }

    pub.publish(msgMod);

    ROS_INFO("SKE -> Published transform with seq numb: %d", msgMod.header.seq);

    if(!countClass)
    {
      ROS_INFO("SKE -> Started");
      countClass = true;
    }

  }
  catch (std::exception e)
  {
      ROS_ERROR("SKE -> No message from detector");
      sleep(3.0);
      return;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "modifier_skeleton_mrroom");
  ros::NodeHandle nh;

  sleep(3.0);

  tf::TransformListener listener2;
  listener = &listener2;

  nh.param("skeleton/kinect_name", kinect, std::string(""));
  ROS_WARN("SKE -> Got param kinect_name: %s", kinect.c_str());
	
  std::string input_track;
  nh.param("skeleton/input_track", input_track, std::string("/tracker/skeleton_tracks"));
  ROS_WARN("SKE -> Got param input_track: %s", input_track.c_str());

  std::string output_topic;
  nh.param("skeleton/output_topic", output_topic, std::string("/arcore/skeleton_tracks"));
  ROS_WARN("MODIFIER SKE -> Got param output_topic: %s", output_topic.c_str());

  pub = nh.advertise<opt_msgs::SkeletonTrackArray>(output_topic.c_str(), 100);
  ros::Subscriber sub = nh.subscribe(input_track.c_str(), 1000, listenerOPTSkeleton);

  ros::spin();

  return 0;
}

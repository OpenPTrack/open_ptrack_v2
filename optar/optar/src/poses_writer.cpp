
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opt_msgs/ARDevicePose.h>
#include <opt_msgs/ARDevicePoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <functional>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <thread>         // std::this_thread::sleep_for

#include "utils.hpp"

using namespace std;

/** ROS node name */
const string NODE_NAME 								= "ardevices_poses_republisher";

const string ardevice_poses_input_topic			= "ardevices_poses_input";
const string tagDetections_topic = "tag_detections";

ofstream devicePoseFileStream;

ofstream tfTag0Kinect01FileStream;
ofstream tfTag1Kinect01FileStream;
ofstream tfTag0Kinect02FileStream;
ofstream tfTag1Kinect02FileStream;

shared_ptr<tf::TransformListener>  listener;

ros::Timer tfCallbackTimer;

geometry_msgs::TransformStamped transformKinect01ToWorld;
geometry_msgs::TransformStamped transformKinect02ToWorld;

void devicePoseCallback(const opt_msgs::ARDevicePoseArrayConstPtr poses)
{
  for(auto p : poses->poses)
  {
    tf::StampedTransform transform;
    try{
      listener->lookupTransform(p.deviceId+"_world_filtered", "/world",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }


    geometry_msgs::PoseStamped phonePose_world;
    geometry_msgs::TransformStamped transform_geom;
    tf::transformStampedTFToMsg(transform,transform_geom);
    tf2::doTransform(poseToPoseStamped(p.pose,"/world", p.header.stamp),phonePose_world,transform_geom);
    devicePoseFileStream << p.deviceId<<";"<<phonePose_world.header.stamp << ";" << poseToString(phonePose_world.pose)<<endl;
    ROS_INFO("Wrote phone pose");
  }

}

void tagDetectionsKinect1Callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& detections)
{
  bool isKinect01 = false;
  if(detections->header.frame_id.compare("kinect01_rgb_optical_frame"))
    isKinect01=true;
  else if(detections->header.frame_id.compare("kinect02_rgb_optical_frame"))
    isKinect01=false;
  else
    throw runtime_error("invalid frame in tag detectionsArray.header");

  for(apriltag_ros::AprilTagDetection atd : detections->detections)
  {
    if(atd.id[0]!=0 && atd.id[0]!=1)
    {
      ROS_WARN("Unknown tag %i, skipping",atd.id[0]);
      continue;
    }
    ofstream* outStream;
    if(isKinect01)
    {
      if(atd.id[0]==0)
        outStream = &tfTag0Kinect01FileStream;
      else if(atd.id[0]==1)
        outStream = &tfTag1Kinect01FileStream;
    }
    else
    {
      if(atd.id[0]==0)
        outStream = &tfTag0Kinect02FileStream;
      else if(atd.id[0]==1)
        outStream = &tfTag1Kinect02FileStream;
    }

    geometry_msgs::PoseStamped tagPose_world;
    tf2::doTransform(poseToPoseStamped(atd.pose.pose.pose,"/world", atd.pose.header.stamp),tagPose_world,transformKinect01ToWorld);

    *outStream << atd.pose.header.stamp << ";" << poseToString(tagPose_world.pose);
    for(float c : atd.pose.pose.covariance)
      *outStream << ";" << c;
    if(isKinect01)
      ROS_INFO("Wrote tag %i pose from kinect01",atd.id[0]);
    else
      ROS_INFO("Wrote tag %i pose from kinect02",atd.id[0]);
  }
}


void tagDetectionsKinect2Callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& detections)
{
  for(apriltag_ros::AprilTagDetection atd : detections->detections)
  {
    ofstream& outStream = (atd.id[0]==0)? tfTag0Kinect02FileStream : tfTag1Kinect02FileStream;
    /*if(atd.id[0]==0)
      outStream = tfTag0Kinect01FileStream;
    else
      outStream = tfTag1Kinect01FileStream;*/

    geometry_msgs::PoseStamped tagPose_world;
    tf2::doTransform(poseToPoseStamped(atd.pose.pose.pose,"/world", atd.pose.header.stamp),tagPose_world,transformKinect02ToWorld);

    outStream << atd.pose.header.stamp << ";" << poseToString(tagPose_world.pose);
    for(float c : atd.pose.pose.covariance)
      outStream << ";" << c;
    ROS_INFO("Wrote tag %i pose from kinect02",atd.id[0]);
  }
}


geometry_msgs::TransformStamped getTransform(const string& inputFrame, const string& targetFrame)
{

  	//get camera to world transform
  	ros::Time targetTime;
  	tf::TransformListener listener;
  	ros::Duration timeout = ros::Duration(10.0);
  	bool retry=true;
  	int count=0;
  	ROS_INFO_STREAM("getting transform from "<<inputFrame<<" to "<<targetFrame);
  	do
  	{
  		std::string failReason;
  		targetTime = ros::Time(0);//the latest available
  		bool r = listener.waitForTransform( targetFrame,inputFrame, targetTime, timeout, ros::Duration(0.01),&failReason);
  		if(!r)
  		{
  			ROS_INFO_STREAM("can't transform because: "<<failReason);
  			if(count>10)
  				throw runtime_error("Cant get transfrom from "+inputFrame+" to "+targetFrame);
  			ROS_INFO("retrying");
  		}
  		else
  			retry=false;
  		count++;
  	}while(retry);
  	tf::StampedTransform transformKinectToWorldNotMsg = tf::StampedTransform();
  	listener.lookupTransform(targetFrame, inputFrame, targetTime, transformKinectToWorldNotMsg);
    geometry_msgs::TransformStamped transformToWorld;
  	tf::transformStampedTFToMsg(transformKinectToWorldNotMsg,transformToWorld);
  	ROS_INFO("Got transform from %s to %s ",inputFrame.c_str(), targetFrame.c_str());
    return transformToWorld;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nodeHandle;
	ROS_INFO_STREAM("starting "<<NODE_NAME);

  std::this_thread::sleep_for (std::chrono::seconds(2));//sleep two second to let tf start
  transformKinect01ToWorld = getTransform("kinect01","/world");
  transformKinect02ToWorld = getTransform("kinect02","/world");

  ros::Subscriber subPoses = nodeHandle.subscribe(ardevice_poses_input_topic, 10, devicePoseCallback);
	ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(ardevice_poses_input_topic));
  ros::Subscriber subTagk1 = nodeHandle.subscribe(tagDetections_topic, 10, tagDetectionsKinect1Callback);
  ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(tagDetections_topic));

  string time = to_string(ros::Time::now().sec);
  string outputPoseFile = "poses"+time;
  string tfTag0Kinect01File = "tfTag0Kinect01"+time;
  string tfTag1Kinect01File = "tfTag1Kinect01"+time;
  string tfTag0kinect02File = "tfTag0Kinect02"+time;
  string tfTag1Kinect02File = "tfTag1Kinect02"+time;

  devicePoseFileStream.open (outputPoseFile);
  tfTag0Kinect01FileStream.open(tfTag0Kinect01File);
  tfTag1Kinect01FileStream.open(tfTag1Kinect01File);
  tfTag0Kinect02FileStream.open(tfTag0kinect02File);
  tfTag1Kinect02FileStream.open(tfTag1Kinect02File);

	ros::spin();

  devicePoseFileStream.close();
  tfTag0Kinect01FileStream.close();
  tfTag1Kinect01FileStream.close();
  tfTag0Kinect02FileStream.close();
  tfTag1Kinect02FileStream.close();
	return 0;
}

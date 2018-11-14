/*
* Author: Daniele Dal Degan [danieledaldegan@gmail.com]
*/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"


class Marker{
  public:
    Marker(std_msgs::Header, std::string);
    void sendPose(std_msgs::Header header, tf::Vector3, tf::Quaternion);

  private:
    visualization_msgs::Marker marker_pose;
};


ros::Publisher vis_pub;
std::string nameTag;
bool countClass = false;
std::map<std::string, Marker> mobilePhones;
tf::Vector3 originTrans;
tf::Quaternion originQuat;


Marker::Marker(std_msgs::Header header, std::string idPhone)
{
  marker_pose.header.frame_id = "world";
  marker_pose.ns = idPhone;
  marker_pose.type = visualization_msgs::Marker::SPHERE;
  marker_pose.action = visualization_msgs::Marker::ADD;
  marker_pose.scale.x = 0.05;
  marker_pose.scale.y = 0.05;
  marker_pose.scale.z = 0.05;
  marker_pose.color.a = 1.0;
  marker_pose.color.r = float(rand()%256) / 255;
  marker_pose.color.g = float(rand()%256) / 255;
  marker_pose.color.b = float(rand()%256) / 255;
  marker_pose.lifetime = ros::Duration(2.0);
}

void Marker::sendPose(std_msgs::Header header, tf::Vector3 tran_input, tf::Quaternion quat_input)
{ 
  marker_pose.header.stamp = header.stamp;
  marker_pose.id = header.seq;
  marker_pose.pose.position.x = tran_input[0];
  marker_pose.pose.position.y = tran_input[1];
  marker_pose.pose.position.z = tran_input[2];
  marker_pose.pose.orientation.x = quat_input[0];
  marker_pose.pose.orientation.y = quat_input[1];
  marker_pose.pose.orientation.z = quat_input[2];
  marker_pose.pose.orientation.w = quat_input[3];

  vis_pub.publish(marker_pose);
}


void listenerVodom(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  try
  {    
    tf::Vector3 tran_arcore = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tf::Quaternion quat_arcore = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);


    tf::Matrix3x3 matrixQuat(tf::Quaternion(originQuat.getX(), originQuat.getY(), originQuat.getZ(), originQuat.getW()));
  	tf::Vector3 transOrig2(originTrans.getX(), originTrans.getY(), originTrans.getZ());

  	tf::Transform transform(matrixQuat, transOrig2);
    transform = transform.inverse();

    // static tf::TransformBroadcaster br;
    // tf::Transform transform;

    // transform.setOrigin( tran_arcore );
    // transform.setRotation( quat_arcore );
    
    // br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, nameTag, msg->header.frame_id));

    for(std::map<std::string, Marker>::iterator iter = mobilePhones.begin(); iter != mobilePhones.end() || (mobilePhones.size() == 0); ++iter)
    {
      if(mobilePhones.find(msg->header.frame_id) == mobilePhones.end())
      {
        mobilePhones.insert(std::map<std::string, Marker>::value_type(msg->header.frame_id, Marker(msg->header, ("mobile_" + mobilePhones.size()))));
        //  mobilePhones[msg->header.frame_id] = marker;
        ROS_INFO("ARCORE -> Phone %s added with id: %lu", (msg->header.frame_id).c_str(), mobilePhones.size());
      }

      if(iter->first.compare(msg->header.frame_id) == 0)
      {
        tf::Matrix3x3 matrixElement(tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w));
        tf::Vector3 translationElement(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        
        tf::Transform transformElement(matrixElement, translationElement);

        tf::Transform multi = transform * transformElement;

        iter->second.sendPose(msg->header, tf::Vector3(multi.getOrigin().x(), multi.getOrigin().y(), multi.getOrigin().z()), tf::Quaternion(multi.getRotation().x(), multi.getRotation().y(), multi.getRotation().z(), multi.getRotation().w()));
      }
    } 

    // Marker marker(msg->header, "mobile_"));
    // marker.sendPose(tran_arcore, quat_arcore);

    if(!countClass)
    {
      ROS_INFO("ARCORE -> Received and published arcore pose");
      countClass = true;
    }

    // ROS_INFO("ARCORE -> Received and published arcore pose seq: %d", msg->header.seq);
  }
  catch (std::exception e)
  {
      ROS_ERROR("ARCORE -> No message received from arcore mobile phone");
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
  ros::init(argc, argv, "Mobile_pose_node");
  ros::NodeHandle nh_mobile;

  sleep(3.0);

  std::string input_name_tag;
  nh_mobile.param("keep_tag_living/name_tag", input_name_tag, std::string("/tag_0_arcore"));
  nameTag = input_name_tag.c_str();
  ROS_WARN("ARCORE -> Got param name_tag: %s", input_name_tag.c_str());

  std::string input_topic;
  nh_mobile.param("mobile_receiver/input_topic", input_topic, std::string("/arcore/vodom"));
  ROS_WARN("ARCORE -> Got param input_topic: %s", input_topic.c_str());

  std::string output_topic;
  nh_mobile.param("mobile_receiver/output_topic", output_topic, std::string("/arcore/marker_array"));
  ROS_WARN("ARCORE -> Got param output_topic: %s", output_topic.c_str());

  std::string origin;
  nh_mobile.param("mobile_receiver/origin", origin, std::string("/arcore/origin"));
  ROS_WARN("MODIFIER -> Got param origin: %s", origin.c_str());

  ros::Subscriber sub = nh_mobile.subscribe(input_topic.c_str(), 10, listenerVodom);
  ros::Subscriber sub2 = nh_mobile.subscribe(origin.c_str(), 10, listenerOrigin);

  vis_pub = nh_mobile.advertise<visualization_msgs::Marker>(output_topic.c_str(), 1);

  ros::spin();

  return 0;
}

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "std_msgs/String.h"

static ros::Publisher vis_pub;
static std::string nameTag;
int countClass = 0;


void publishMarker(std_msgs::Header header, tf::Vector3 tran_input, tf::Quaternion quat_input)
{
  visualization_msgs::Marker marker_pose;
  marker_pose.header.frame_id = nameTag;
  marker_pose.header.stamp = ros::Time::now();
  marker_pose.ns = "mobile_phone";
  marker_pose.id = header.seq;
  marker_pose.type = visualization_msgs::Marker::SPHERE;
  marker_pose.action = visualization_msgs::Marker::ADD;
  marker_pose.pose.position.x = tran_input[0];
  marker_pose.pose.position.y = tran_input[1];
  marker_pose.pose.position.z = tran_input[2];
  marker_pose.pose.orientation.x = quat_input[0];
  marker_pose.pose.orientation.y = quat_input[1];
  marker_pose.pose.orientation.z = quat_input[2];
  marker_pose.pose.orientation.w = quat_input[3];
  marker_pose.scale.x = 0.05;
  marker_pose.scale.y = 0.05;
  marker_pose.scale.z = 0.05;
  marker_pose.color.a = 1.0;
  marker_pose.color.r = 0.33;
  marker_pose.color.g = 0.94;
  marker_pose.color.b = 0.20;
  marker_pose.lifetime = ros::Duration(3.0);

  vis_pub.publish(marker_pose);
}


void listenerVodom(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  try
  {    
    tf::Vector3 tran_arcore = tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    tf::Quaternion quat_arcore = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin( tran_arcore );
    transform.setRotation( quat_arcore );
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), nameTag, msg->header.frame_id));
        
    publishMarker(msg->header, tran_arcore, quat_arcore);

    if(countClass == 0)
    {
      ROS_INFO("ARCORE -> Received and published arcore pose");
      countClass++;
    }

    // ROS_INFO("ARCORE -> Received and published arcore pose seq: %d", msg->header.seq);
  }
  catch (std::exception e)
  {
      ROS_ERROR("ARCORE -> No message received from arcore mobile phone");
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

  ros::Subscriber sub = nh_mobile.subscribe(input_topic.c_str(), 1000, listenerVodom);
  vis_pub = nh_mobile.advertise<visualization_msgs::Marker>(output_topic.c_str(), 0);

  ros::spin();

  return 0;
}

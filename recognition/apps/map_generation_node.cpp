#include <mutex>
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

class MapGenerationNode {
public:
  MapGenerationNode(ros::NodeHandle& nh)
    : timer(nh.createTimer(ros::Duration(2.0), &MapGenerationNode::spin_once, this)),
      map_pub(nh.advertise<nav_msgs::OccupancyGrid>("/map", 10))
  {
    accumulated.reset(new pcl::PointCloud<pcl::PointXYZ>());
    find_sensors(nh);
  }

  void spin_once(const ros::TimerEvent& event) {
    auto map_msg = generate_map();
    if(map_msg == nullptr) {
      return;
    }

    map_pub.publish(map_msg);

    /*
    std::lock_guard<std::mutex> lock(mutex);
    vis.removeAllPointClouds();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler(accumulated, 255.0, 0.0, 0.0);
    vis.addPointCloud(accumulated, handler);
    vis.spinOnce(10);
    */
  }

  nav_msgs::OccupancyGridPtr generate_map() {
    std::cout << "generate" << std::endl;
    double m_per_cell = 0.05;
    int width = 1024;
    int height = 1024;

    std::unique_lock<std::mutex> lock(mutex);
    if(accumulated->empty()) {
      return nullptr;
    }

    nav_msgs::OccupancyGridPtr map_msg(new nav_msgs::OccupancyGrid());
    map_msg->header.frame_id = "map";
    map_msg->header.stamp = ros::Time::now();

    map_msg->info.map_load_time = map_msg->header.stamp;
    map_msg->info.resolution = m_per_cell;
    map_msg->info.width = width;
    map_msg->info.height = height;
    map_msg->info.origin.position.x = map_msg->info.origin.position.y = map_msg->info.origin.position.z = 0.0;
    map_msg->info.origin.orientation.x = map_msg->info.origin.orientation.y = map_msg->info.origin.orientation.z = 0.0;
    map_msg->info.origin.orientation.w = 1.0;

    map_msg->data.resize(width * height);

    for(const auto& pt : accumulated->points) {
      int x = pt.x / m_per_cell + width / 2;
      int y = pt.y / m_per_cell + height / 2;

      if(x >= 0 && x < width && y >= 0 && y < height) {
        auto& pix = map_msg->data[y * width + x];
        pix = std::min<int>(100, pix + 1);
      }
    }

    lock.unlock();

    cv::Mat image(height, width, CV_8UC1, map_msg->data.data());
    cv::imshow("image", image * 2);
    cv::waitKey(20);

    return map_msg;
  }

private:
  void find_sensors(ros::NodeHandle& nh) {
    std::vector<std::string> sensor_names = {"kinect2_far", "kinect2_lenovo"};

    points_subs.resize(sensor_names.size());
    for(int i=0; i<sensor_names.size(); i++) {
      std::string topic = "/" + sensor_names[i] + "/depth_ir/points";
      points_subs[i] = nh.subscribe(topic, 10, &MapGenerationNode::points_callback, this);
    }
  }

  void points_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    if(cloud->empty()) {
      std::cout << "empty cloud!!" << std::endl;
      return;
    }

    auto& prev_time = last_observation_times[cloud->header.frame_id];
    if((ros::Time::now() - prev_time) < ros::Duration(0.5)) {
      std::cout << "skip" << std::endl;
      return;
    }
    prev_time = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_ros::transformPointCloud("/world", *cloud, *transformed, tf_listener);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    std::copy_if(transformed->begin(), transformed->end(), std::back_inserter(filtered->points),
      [=](const pcl::PointXYZ& pt) {
        return 0.2 < pt.z && pt.z < 0.5;
      }
    );

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered);
    sor.setMeanK(25);
    sor.setStddevMulThresh(1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed(new pcl::PointCloud<pcl::PointXYZ>());
    sor.filter(*outlier_removed);

    std::lock_guard<std::mutex> lock(mutex);
    std::copy(outlier_removed->begin(), outlier_removed->end(), std::back_inserter(accumulated->points));
  }

private:
  ros::Timer timer;
  ros::Publisher map_pub;

  ros::ServiceClient service_client;

  tf::TransformListener tf_listener;
  std::vector<ros::Subscriber> points_subs;
  std::unordered_map<std::string, ros::Time> last_observation_times;

  std::mutex mutex;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated;

//  pcl::visualization::PCLVisualizer vis;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "map_generation_node");
  ros::NodeHandle nh;

  MapGenerationNode node(nh);
  ros::spin();

  return 0;
}

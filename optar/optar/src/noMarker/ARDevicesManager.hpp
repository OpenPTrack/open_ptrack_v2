#ifndef AR_DEVICES_MANAGER_HPP_201905
#define AR_DEVICES_MANAGER_HPP_201905

#include <string>
#include <chrono>
#include <map>
#include <memory>
#include <geometry_msgs/PoseStamped.h>
#include <functional>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>

using namespace std;


class ARDevicesManager
{

public:
  class Device
  {
  private:
    ros::Subscriber posesSubscriber;
    ARDevicesManager* devicesManager;
  public:
    struct DeviceInfo
    {
      std::string arDeviceId;
      ros::Time lastSeenAlive;
    	geometry_msgs::PoseStampedConstPtr lastPoseMessage;
      bool didEverReceivePose;
    };
    DeviceInfo deviceInfo;


    void signalDeviceAlive();
    std::string getARDeviceId();
    void poseMsgCallback(const geometry_msgs::PoseStampedConstPtr poseMsg);
    Device(std::string arDeviceId, shared_ptr<ros::NodeHandle> nodeHandle, bool listenToPoses, ARDevicesManager* devicesManager);
    ~Device();
  };

private:

  ros::Duration deviceAliveTimeout;
  bool listenToPoses;
  std::function<void(const std::string&, const geometry_msgs::PoseStampedConstPtr&)> onPoseReceivedCallback;
  std::function<void(const std::string&)> onDeviceConnected;
  std::function<void(const std::string&)> onDeviceDisconnected;
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  std::timed_mutex aliveDevicesMutex;
  map<string, shared_ptr<Device>> aliveDevices;
  char lastLockingMethod[50] = {0};
  ros::Timer updateCallerTimer;
  ros::Subscriber heartbeatsSubscriber;

  std::string devices_heartbeats_topicName = "heartbeats_topic";

  void updateCallback(const ros::TimerEvent&);
  void cleanupDeadDevices();
  void heartbeatCallback(const std_msgs::StringConstPtr& msg);

public:


  ARDevicesManager(bool listenToPoses, const ros::Duration& deviceAliveTimeout);
  void setupParameters(const ros::Duration& deviceAliveTimeout);
  void start(std::shared_ptr<ros::NodeHandle> nodeHandle);
  void signalDeviceAlive(std::string arDeviceId);
  bool isDeviceAlive(std::string arDeviceId);
  void setOnPoseReceivedCallback(std::function<void(const std::string&,const geometry_msgs::PoseStampedConstPtr&)> callback);
  void setOnDeviceConnected(std::function<void(const std::string&)> callback);
  void setOnDeviceDisconnected(std::function<void(const std::string&)> callback);
  std::vector<Device::DeviceInfo> getAliveDevicesInfo();
};


#endif

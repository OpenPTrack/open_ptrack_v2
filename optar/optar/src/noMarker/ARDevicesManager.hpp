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

/**
 * Class for managing the availabel AR devices
 */
class ARDevicesManager
{

public:
  /**
   * Reperesentation of an AR device
   */
  class Device
  {
  private:
    /** Subscriber for receiving the ARCore poses fromt he device */
    ros::Subscriber posesSubscriber;
    /** Pointer to the device manager that is managing the device */
    ARDevicesManager* devicesManager;
  public:
    /**
     * Details of the device
     */
    struct DeviceInfo
    {
      /** unique id of the device */
      std::string arDeviceId;
      /** The ast time the device was seen to be active */
      ros::Time lastSeenAlive;
      /** The last ARCore pose message that was received */
    	geometry_msgs::PoseStampedConstPtr lastPoseMessage;
      /** True if an ARCore pose was ever received */
      bool didEverReceivePose;
    };
    /** Contains the details of the device */
    DeviceInfo deviceInfo;

    void signalDeviceAlive();
    std::string getARDeviceId();
    void poseMsgCallback(const geometry_msgs::PoseStampedConstPtr poseMsg);
    Device(std::string arDeviceId, shared_ptr<ros::NodeHandle> nodeHandle, bool listenToPoses, ARDevicesManager* devicesManager);
    ~Device();
  };

private:

  /** Timeout after which a device that is not active is removed by the manager memory */
  ros::Duration deviceAliveTimeout;
  /** Indicates if the pose listening is enabled */
  bool listenToPoses;
  /** Pointer to an external callback called when a new device pose is received */
  std::function<void(const std::string&, const geometry_msgs::PoseStampedConstPtr&)> onPoseReceivedCallback;
  /** Pointer to an external callback called when a new device connects */
  std::function<void(const std::string&)> onDeviceConnected;
  /** Pointer to an external callback called when a device disconnects */
  std::function<void(const std::string&)> onDeviceDisconnected;
  /** NodeHandle for the current ROS node */
  std::shared_ptr<ros::NodeHandle> nodeHandle;

  /** Mutex for the connected devices memory #aliveDevices*/
  std::timed_mutex aliveDevicesMutex;
  /** Memory for the connected devices */
  map<string, shared_ptr<Device>> aliveDevices;
  /** For debug: name of the last method that acquire the mutex */
  char lastLockingMethod[50] = {0};
  /** Timer for periodically calling the #updateCallback method */
  ros::Timer updateCallerTimer;
  /** Subscriber for the device heartbeats */
  ros::Subscriber heartbeatsSubscriber;

  /** Name of the devices heartbeats topic */
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

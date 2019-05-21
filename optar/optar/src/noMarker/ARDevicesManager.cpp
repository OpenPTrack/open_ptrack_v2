#include "ARDevicesManager.hpp"

ARDevicesManager::ARDevicesManager(bool listenToPoses, const ros::Duration& deviceAliveTimeout)
{
  this->listenToPoses = listenToPoses;
  setupParameters(deviceAliveTimeout);
}

void ARDevicesManager::setupParameters(const ros::Duration& deviceAliveTimeout)
{
  this->deviceAliveTimeout = deviceAliveTimeout;
}

void ARDevicesManager::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
  heartbeatsSubscriber = nodeHandle->subscribe(devices_heartbeats_topicName, 10,
                                               &ARDevicesManager::heartbeatCallback, this);
  ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(devices_heartbeats_topicName));

  this->nodeHandle = nodeHandle;

  updateCallerTimer = nodeHandle->createTimer(ros::Duration(1), &ARDevicesManager::updateCallback, this);
}

void ARDevicesManager::signalDeviceAlive(std::string arDeviceId)
{
  std::unique_lock<std::timed_mutex> lock(aliveDevicesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
    ROS_ERROR_STREAM("ARDevicesManager::"<<__func__<<": failed to get mutex. Aborting. last = "<<lastLockingMethod);
		return;
	}
  std::string str( "signalDeviceAlive" );

  str.copy(lastLockingMethod, 49);
  lastLockingMethod[str.length()] = '\0';

  auto it = aliveDevices.find(arDeviceId);
  if(it==aliveDevices.end())//if it doesn't exist, create it
  {
    shared_ptr<Device> newDevice = std::make_shared<Device>(arDeviceId,nodeHandle, listenToPoses, this);

    aliveDevices.insert(std::map<string, shared_ptr<Device>>::value_type(newDevice->getARDeviceId(),newDevice));
    ROS_INFO_STREAM("Detected new device "<<newDevice->getARDeviceId());
    onDeviceConnected(arDeviceId);
  }
  aliveDevices.find(arDeviceId)->second->signalDeviceAlive();
}

void ARDevicesManager::heartbeatCallback(const std_msgs::StringConstPtr& msg)
{
  string deviceName = msg->data;
  signalDeviceAlive(deviceName);
}


void ARDevicesManager::cleanupDeadDevices()
{
  ROS_INFO_STREAM("starting cleanupDeadDevices()");
  std::unique_lock<std::timed_mutex> lock(aliveDevicesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
    ROS_ERROR_STREAM("ARDevicesManager::"<<__func__<<": failed to get mutex. Aborting. last = "<<lastLockingMethod);
		return;
	}
  ROS_INFO_STREAM("cleanupDeadDevices(): got mutex");
  std::string str( "cleanupDeadDevices" );

  str.copy(lastLockingMethod, 49);
  lastLockingMethod[str.length()] = '\0';

  ros::Time now = ros::Time::now();
  auto it = aliveDevices.cbegin();
  while(it != aliveDevices.end())
  {
    if(now - it->second->deviceInfo.lastSeenAlive > deviceAliveTimeout)
    {
      std::string arDeviceId = it->second->deviceInfo.arDeviceId;
      it = aliveDevices.erase(it);
      onDeviceDisconnected(arDeviceId);
    }
    else
    {
      it++;
    }
  }
  ROS_INFO_STREAM("cleanupDeadDevices(): finished");
}




void ARDevicesManager::updateCallback(const ros::TimerEvent&)
{
  cleanupDeadDevices();
}



bool ARDevicesManager::isDeviceAlive(std::string arDeviceId)
{
  std::unique_lock<std::timed_mutex> lock(aliveDevicesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
    ROS_ERROR_STREAM("ARDevicesManager::"<<__func__<<": failed to get mutex. Aborting. last = "<<lastLockingMethod);
		return false;
	}
  std::string str( "isDeviceAlive" );
  str.copy(lastLockingMethod, 49);
  lastLockingMethod[str.length()] = '\0';

  return aliveDevices.find(arDeviceId) != aliveDevices.end();
}



std::vector<ARDevicesManager::Device::DeviceInfo> ARDevicesManager::getAliveDevicesInfo()
{
  std::vector<Device::DeviceInfo> ret;
  std::unique_lock<std::timed_mutex> lock(aliveDevicesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM("ARDevicesManager::"<<__func__<<": failed to get mutex. Aborting. last = "<<lastLockingMethod);
		return ret;
	}
  std::string str( "getAliveDevicesInfo" );
  str.copy(lastLockingMethod, 49);
  lastLockingMethod[str.length()] = '\0';

  for (auto it = aliveDevices.cbegin(); it != aliveDevices.cend();)
    ret.push_back(it->second->deviceInfo);
  return ret;
}



void ARDevicesManager::setOnDeviceConnected(std::function<void(const std::string&)> callback)
{
  onDeviceConnected = callback;
}


void ARDevicesManager::setOnDeviceDisconnected(std::function<void(const std::string&)> callback)
{
  onDeviceDisconnected = callback;
}


void ARDevicesManager::setOnPoseReceivedCallback(std::function<void(const std::string&, const geometry_msgs::PoseStampedConstPtr&)> callback)
{
  onPoseReceivedCallback = callback;
}







ARDevicesManager::Device::Device(std::string arDeviceId,
                                 shared_ptr<ros::NodeHandle> nodeHandle,
                                 bool listenToPoses,
                                 ARDevicesManager* devicesManager)
{
  deviceInfo.arDeviceId = arDeviceId;
  deviceInfo.lastSeenAlive = ros::Time::now();
  deviceInfo.didEverReceivePose = false;
  this->devicesManager = devicesManager;

  if(listenToPoses)
    posesSubscriber = nodeHandle->subscribe("/optar/"+arDeviceId+"/pose", 10, &ARDevicesManager::Device::poseMsgCallback, this);


}



void ARDevicesManager::Device::poseMsgCallback(const geometry_msgs::PoseStampedConstPtr poseMsg)
{
  signalDeviceAlive();
  deviceInfo.lastPoseMessage = poseMsg;
  deviceInfo.didEverReceivePose = true;
  devicesManager->onPoseReceivedCallback(deviceInfo.arDeviceId, poseMsg);
}


ARDevicesManager::Device::~Device()
{
  posesSubscriber.shutdown();
}


void ARDevicesManager::Device::signalDeviceAlive()
{
  deviceInfo.lastSeenAlive = ros::Time::now();
}

std::string ARDevicesManager::Device::getARDeviceId()
{
  return deviceInfo.arDeviceId;
}

#include "ARDevicesManager.hpp"

/**
 * Constructs the device manager
 * @param listenToPoses      Set to true if you want to listen to the ARCore poses
 * @param deviceAliveTimeout Timeout after which inactive devices are forgotten
 */
ARDevicesManager::ARDevicesManager(bool listenToPoses, const ros::Duration& deviceAliveTimeout)
{
  this->listenToPoses = listenToPoses;
  setupParameters(deviceAliveTimeout);
}

/**
 * Update the parameteres of the device manager
 * @param deviceAliveTimeout Timeout after which inactive devices are forgotten
 */
void ARDevicesManager::setupParameters(const ros::Duration& deviceAliveTimeout)
{
  this->deviceAliveTimeout = deviceAliveTimeout;
}

/**
 * Starts the device manager, starts listening for active devices
 * @param nodeHandle NodeHandle for the current ROS node
 */
void ARDevicesManager::start(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
  heartbeatsSubscriber = nodeHandle->subscribe(devices_heartbeats_topicName, 10,
                                               &ARDevicesManager::heartbeatCallback, this);
  ROS_INFO_STREAM("Subscribed to "<<ros::names::remap(devices_heartbeats_topicName));

  this->nodeHandle = nodeHandle;

  updateCallerTimer = nodeHandle->createTimer(ros::Duration(1), &ARDevicesManager::updateCallback, this);
}

/**
 * Signals that the device is active, updating the deviceInfo accordingly
 */
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

/**
 * Calaback that receoves the devices heartbeats, with which the active devices are monitored
 * @param msg The received heartbeat message
 */
void ARDevicesManager::heartbeatCallback(const std_msgs::StringConstPtr& msg)
{
  string deviceName = msg->data;
  signalDeviceAlive(deviceName);
}

/**
 * Cleans up the manager memory from inactive devices, stopping the handlers
 * for the inactive devices
 */
void ARDevicesManager::cleanupDeadDevices()
{
  //ROS_INFO_STREAM("starting cleanupDeadDevices()");
  std::unique_lock<std::timed_mutex> lock(aliveDevicesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
    ROS_ERROR_STREAM("ARDevicesManager::"<<__func__<<": failed to get mutex. Aborting. last = "<<lastLockingMethod);
		return;
	}
  //ROS_INFO_STREAM("cleanupDeadDevices(): got mutex");
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
  //ROS_INFO_STREAM("cleanupDeadDevices(): finished");
}



/**
 * Update method called periodically
 */
void ARDevicesManager::updateCallback(const ros::TimerEvent&)
{
  cleanupDeadDevices();
}


/**
 * Checks if the device with the provided id is active
 * @param  arDeviceId ID of the device
 * @return            True if it is active
 */
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


/**
 * Returns the device information for all the active devices
 */
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


/**
 * Sets the callback for listening to the newly connected devices
 * @param callback method that will be called
 */
void ARDevicesManager::setOnDeviceConnected(std::function<void(const std::string&)> callback)
{
  onDeviceConnected = callback;
}


/**
 * Sets the callback for listening to the evice disconnections
 * @param callback method that will be called
 */
void ARDevicesManager::setOnDeviceDisconnected(std::function<void(const std::string&)> callback)
{
  onDeviceDisconnected = callback;
}


/**
 * Sets the callback for listening to the newly received device  ARCore poses
 * @param callback method that will be called
 */
void ARDevicesManager::setOnPoseReceivedCallback(std::function<void(const std::string&, const geometry_msgs::PoseStampedConstPtr&)> callback)
{
  onPoseReceivedCallback = callback;
}






/**
 * Constructs the object with the provided details
 * @param arDeviceId     unique ID of the device
 * @param nodeHandle     ROS node handle for the current ROS node
 * @param listenToPoses  true to enable the pose listener
 * @param devicesManager the ARDevicesManager that is handling the device
 */
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



/** Callback executed when a new ARCore pose message is received
 * @param poseMsg The message that was received
 */
void ARDevicesManager::Device::poseMsgCallback(const geometry_msgs::PoseStampedConstPtr poseMsg)
{
  signalDeviceAlive();
  deviceInfo.lastPoseMessage = poseMsg;
  deviceInfo.didEverReceivePose = true;
  devicesManager->onPoseReceivedCallback(deviceInfo.arDeviceId, poseMsg);
}

/**
 * Destructor for the device object. It stops the interanl subscribers
 */
ARDevicesManager::Device::~Device()
{
  posesSubscriber.shutdown();
}

/**
 * Noted that the device is alive by updating the Device::DeviceInfo::lastSeenAlive member
 */
void ARDevicesManager::Device::signalDeviceAlive()
{
  deviceInfo.lastSeenAlive = ros::Time::now();
}

/** Returns the unique id of the device
*/
std::string ARDevicesManager::Device::getARDeviceId()
{
  return deviceInfo.arDeviceId;
}

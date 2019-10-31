/**
 * @file
 *
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 *
 * FeatureMemory methods implementation file
 */


#include "FeaturesMemory.hpp"
#include <cmath>
#include <chrono>
#include <thread>         // std::this_thread::sleep_for
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;


/**
 * Save the provided feature in memory
 * @param newFeature The feature to be saved
 */
void FeaturesMemory::saveFeature(const Feature& newFeature)
{
	std::unique_lock<std::timed_mutex> lock(savedFeaturesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Can't save feature");
		return;
	}

	std::vector<Feature*> equivalentFeatures;
	for(shared_ptr<Feature>& savedFeature : savedFeatures)
	{
		if(	newFeature.pixelDistance(*savedFeature) <= featureEquivalencePositionThreshold_pixels &&
			newFeature.observerAngularDistance(*savedFeature) <= featureEquivalenceAngularDistanceThreshold_radiants &&
			newFeature.observerDistanceDifference(*savedFeature) <= featureEquivalenceObserverDistanceThreshold_meters &&
			newFeature.descriptorDistance(*savedFeature) <= equivalentDescriptorsMaximumDistance &&
			newFeature.keypoint.octave == savedFeature->keypoint.octave)
		{
			equivalentFeatures.push_back(&(*savedFeature));// bleah
		}
	}

	ROS_INFO_STREAM("found "<<equivalentFeatures.size()<<" equivalent features");
	for(Feature* equivalentFeature : equivalentFeatures)//confirm all the equivalent features
	{
		equivalentFeature->timesConfirmed++;
	}

	if(equivalentFeatures.size()==0)// if there is no equivalent feature add it
	{
		savedFeatures.push_back(make_shared<FeaturesMemory::Feature>(newFeature));
	}
}

/**
 * Saves a list of features to memory
 * @param newFeatures The new features
 */
void FeaturesMemory::saveFeatures(const std::vector<Feature>& newFeatures)
{
	for(const Feature& newFeature : newFeatures)
	{
		saveFeature(newFeature);
	}
}

/**
 * Removes memorized features that are not on the background.
 * This is done by removing features that are much coser to the camera than the
 * depth reported on the provided image states.
 * @param depthImage Depth image from the camera
 */
void FeaturesMemory::removeNonBackgroundFeatures(const cv::Mat& depthImage)
{
	for(size_t i=0;i<savedFeatures.size();i++)
	{
		const Feature& feature = *(savedFeatures.at(i));
		if(feature.depth_mm - depthImage.at<uint16_t>(feature.keypoint.pt) < -featureDepthMatchThreshold_mm)
		{
			savedFeatures.erase(savedFeatures.begin() + i);
			i--;
		}
	}
}

/**
 * Gets all the features from the memory
 */
const std::vector<FeaturesMemory::Feature> FeaturesMemory::getFeatures()
{
	std::vector<FeaturesMemory::Feature> savedFeaturesCopy;
	std::unique_lock<std::timed_mutex> lock(savedFeaturesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Can't get features");
		return savedFeaturesCopy;
	}

	for(const shared_ptr<Feature>& savedFeature : savedFeatures)
	{
		savedFeaturesCopy.push_back(Feature(*savedFeature));
	}
	return savedFeaturesCopy;
}

/**
 * Computes the distance pixel distance in an image from this feature and another one
 * TODO: this doesn't make sense, should change to the 3d position
 * @param feature The other feature
 */
bool FeaturesMemory::Feature::pixelDistance(const FeaturesMemory::Feature& feature) const
{
	return sqrt((feature.keypoint.pt.x - keypoint.pt.x)*(feature.keypoint.pt.x - keypoint.pt.x) - (feature.keypoint.pt.y - keypoint.pt.y)*(feature.keypoint.pt.y - keypoint.pt.y));
}

/**
 * Angular distance between the directions of the two observers of these two features
 * @param feature the other feature
 */
double FeaturesMemory::Feature::observerAngularDistance(const FeaturesMemory::Feature& feature) const
{
	tf::Vector3 observerDirTf(observerDirection.x,observerDirection.y,observerDirection.z);
	tf::Vector3 otherObserverDirTf(feature.observerDirection.x,feature.observerDirection.y,feature.observerDirection.z);
	return std::abs(observerDirTf.angle(otherObserverDirTf));
}

/**
 * Difference between the distances of the observer in this feature and the distance
 * of the observer in another feature
 * @param feature The other feature
 */
double FeaturesMemory::Feature::observerDistanceDifference(const FeaturesMemory::Feature& feature) const
{
	return observerDistance_meters - feature.observerDistance_meters;
}

/**
 * Distance between the feature descriptors of this feature and another one
 * @param feature The other feature
 */
unsigned int FeaturesMemory::Feature::descriptorDistance(const FeaturesMemory::Feature& feature) const
{
	return norm( descriptor, feature.descriptor, cv::NORM_HAMMING);
}

/**
 * Builds a Feature object
 * @param keypoint                Keypoint of this feature
 * @param descriptor              Descriptor of this feature
 * @param observerDistance_meters Distance of the observer
 * @param observerDirection       Direction of the observer
 * @param depth_mm                Depth of the feature in the 2d image
 */
FeaturesMemory::Feature::Feature(const cv::KeyPoint& keypoint, const cv::Mat& descriptor, double observerDistance_meters, const cv::Point3f& observerDirection, double depth_mm) :
	keypoint(keypoint),
	descriptor(descriptor),
	observerDistance_meters(observerDistance_meters),
	observerDirection(observerDirection),
	depth_mm(depth_mm)
{
	timesConfirmed = 1;

	if(descriptor.cols!=descriptorSize)
		throw invalid_argument("descriptor has invalid size, should be "+to_string(descriptorSize)+" but is "+to_string(descriptor.cols));
}



/**
 * Copies a feature object
 * @param feature The feature object to be copied
 */
FeaturesMemory::Feature::Feature(const FeaturesMemory::Feature& feature) :
	Feature(feature.keypoint, feature.descriptor, feature.observerDistance_meters, feature.observerDirection, feature.depth_mm)
{
	timesConfirmed = feature.timesConfirmed;

	if(descriptor.cols!=descriptorSize)
		throw invalid_argument("descriptor has invalid size, should be "+to_string(descriptorSize)+" but is "+to_string(descriptor.cols));
}

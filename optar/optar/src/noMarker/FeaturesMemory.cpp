#include "FeaturesMemory.hpp"
#include <cmath>
#include <chrono>
#include <thread>         // std::this_thread::sleep_for
#include <mutex>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

void FeaturesMemory::saveFeature(const Feature& newFeature)
{
	std::unique_lock<std::timed_mutex> lock(savedFeaturesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Can't save feature");
		return;
	}

	std::vector<Feature*> equivalentFeatures;
	for(Feature& savedFeature : savedFeatures)
	{
		if(	newFeature.pixelDistance(savedFeature) <= featureEquivalencePositionThreshold_pixels &&
			newFeature.observerAngularDistance(savedFeature) <= featureEquivalenceAngularDistanceThreshold_radiants && 
			newFeature.observerDistanceDifference(savedFeature) <= featureEquivalenceObserverDistanceThreshold_meters && 
			newFeature.descriptorDistance(savedFeature) <= equivalentDescriptorsMaximumDistance &&
			newFeature.keypoint.octave == savedFeature.keypoint.octave)
		{
			equivalentFeatures.push_back(&savedFeature);
		}
	}	

	ROS_INFO_STREAM("found "<<equivalentFeatures.size()<<" equivalent features");
	for(Feature* equivalentFeature : equivalentFeatures)//confirm all the equivalent features
	{
		equivalentFeature->timesConfirmed++;
	}

	if(equivalentFeatures.size()==0)// if there is no equivalent feature add it
	{
		savedFeatures.push_back(newFeature);
	}
}


void FeaturesMemory::saveFeatures(const std::vector<Feature>& newFeatures)
{
	for(const Feature& newFeature : newFeatures)
	{
		saveFeature(newFeature);
	}
}


const std::vector<FeaturesMemory::Feature> FeaturesMemory::getFeatures()
{
	std::vector<FeaturesMemory::Feature> savedFeaturesCopy;
	std::unique_lock<std::timed_mutex> lock(savedFeaturesMutex, std::chrono::milliseconds(5000));
	if(!lock.owns_lock())
	{
		ROS_ERROR_STREAM(""<<__func__<<": failed to get mutex. Can't get features");
		return savedFeaturesCopy;
	}

	for(const Feature& savedFeature : savedFeatures)
	{
		savedFeaturesCopy.push_back(Feature(savedFeature));
	}	
	return savedFeaturesCopy;
}


bool FeaturesMemory::Feature::pixelDistance(const FeaturesMemory::Feature& feature) const
{
	return sqrt((feature.keypoint.pt.x - keypoint.pt.x)*(feature.keypoint.pt.x - keypoint.pt.x) - (feature.keypoint.pt.y - keypoint.pt.y)*(feature.keypoint.pt.y - keypoint.pt.y));
}

double FeaturesMemory::Feature::observerAngularDistance(const FeaturesMemory::Feature& feature) const
{
	tf::Vector3 observerDirTf(observerDirection.x,observerDirection.y,observerDirection.z);
	tf::Vector3 otherObserverDirTf(feature.observerDirection.x,feature.observerDirection.y,feature.observerDirection.z);
	return std::abs(observerDirTf.angle(otherObserverDirTf));
}

double FeaturesMemory::Feature::observerDistanceDifference(const FeaturesMemory::Feature& feature) const
{
	return observerDistance_meters - feature.observerDistance_meters;
}

unsigned int FeaturesMemory::Feature::descriptorDistance(const FeaturesMemory::Feature& feature) const
{
	return norm( descriptor, feature.descriptor, cv::NORM_HAMMING);
}


FeaturesMemory::Feature::Feature(const cv::KeyPoint& keypoint, const cv::Mat& descriptor, double observerDistance_meters, const cv::Point3f& observerDirection) : 
	keypoint(keypoint),
	descriptor(descriptor),
	observerDistance_meters(observerDistance_meters),
	observerDirection(observerDirection)
{
	timesConfirmed = 1;

	if(descriptor.cols!=descriptorSize)
		throw invalid_argument("descriptor has invalid size, should be "+to_string(descriptorSize)+" but is "+to_string(descriptor.cols));
}

FeaturesMemory::Feature::Feature(const FeaturesMemory::Feature& feature) : 
	keypoint(feature.keypoint),
	descriptor(feature.descriptor),
	observerDistance_meters(feature.observerDistance_meters),
	observerDirection(feature.observerDirection)
{
	timesConfirmed = feature.timesConfirmed;

	if(descriptor.cols!=descriptorSize)
		throw invalid_argument("descriptor has invalid size, should be "+to_string(descriptorSize)+" but is "+to_string(descriptor.cols));
}

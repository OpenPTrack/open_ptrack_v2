#ifndef FEATURES_MEMORY_HPP_20190328
#define FEATURES_MEMORY_HPP_20190328


#include <opencv2/features2d/features2d.hpp>
#include <mutex>

class FeaturesMemory
{
public:
	class Feature
	{
	public:
		const int descriptorSize = 32;
		const cv::KeyPoint keypoint;
		const cv::Mat descriptor;
		const double observerDistance_meters;
		const cv::Point3f observerDirection;

		unsigned int timesConfirmed; //the idea is to keep track of how many times this feature has been confirmed by equivalent savings

		bool pixelDistance(const Feature& feature) const;
		double observerAngularDistance(const Feature& feature) const;
		double observerDistanceDifference(const Feature& feature) const;
		unsigned int descriptorDistance(const Feature& feature) const;

		Feature(const cv::KeyPoint& keypoint, const cv::Mat& descriptor, double observerDistance_meters, const cv::Point3f& observerDirection);

		Feature(const Feature& feature);

	};


private:
	std::vector<FeaturesMemory::Feature> savedFeatures;
	std::timed_mutex savedFeaturesMutex;

	double featureEquivalencePositionThreshold_pixels = 1;
	double featureEquivalenceAngularDistanceThreshold_radiants = 3.14159/8;//22.5 degrees
	double featureEquivalenceObserverDistanceThreshold_meters = 1;//maybe this should not be a constant threshold, as a difference from 0.1 meters to 1.1 is very differnet form a difference from 10 meters to 11
	unsigned int equivalentDescriptorsMaximumDistance = 20;

public:
	void saveFeature(const FeaturesMemory::Feature& newFeature);
	void saveFeatures(const std::vector<FeaturesMemory::Feature>& newFeatures);

	const std::vector<FeaturesMemory::Feature> getFeatures();
};


#endif
/**
 * @file
 *
 * @author Carlo Rizzardo (crizz, cr.git.mail@gmail.com)
 *
 * FeaturesMemory class declaration file
 */

#ifndef FEATURES_MEMORY_HPP_20190328
#define FEATURES_MEMORY_HPP_20190328


#include <opencv2/features2d/features2d.hpp>
#include <mutex>
#include <memory>

/**
 * Class for keeping track of features in the scene.
 * It saves features seen by the mobile devices, together with information of the
 * position of the observer. The idea is to keep track of keypoints and save feature
 * descriptors representing them from different points of view.
 * Descriptors that are very similar and seen from similar points of view should
 * be merged in a single descriptor, while descriptors build from different perspectives
 * should be kept separate.
 */
class FeaturesMemory
{
public:
	/**
	 * Describes a bidimensional feature
	 */
	class Feature
	{
	public:
		/** Size of the ORB descriptors */
		const int descriptorSize = 32;

		/** 2d keypoint for this descriptor. TODO: Not really that useful, we should use the 3d position */
		const cv::KeyPoint keypoint;
		/** ORB descriptor */
		const cv::Mat descriptor;
		/** Distance of the observer */
		const double observerDistance_meters;
		/** Direction of the observer */
		const cv::Point3f observerDirection;
		/** depth of this feature in the 2d image (in millimeters). Useful to distinguish features on the background from features on moving objects*/
		const double depth_mm;

		/** Number of times we have seen this feature (or very similiar ones) */
		unsigned int timesConfirmed;

		bool pixelDistance(const Feature& feature) const;
		double observerAngularDistance(const Feature& feature) const;
		double observerDistanceDifference(const Feature& feature) const;
		unsigned int descriptorDistance(const Feature& feature) const;

		Feature(const cv::KeyPoint& keypoint, const cv::Mat& descriptor, double observerDistance_meters, const cv::Point3f& observerDirection, double depth_mm);

		Feature(const Feature& feature);

	};


private:
	/** List of the features we are memorizing */
	std::vector<std::shared_ptr<FeaturesMemory::Feature>> savedFeatures;//TODO: use something with better add/remove performance. Mainly because of the removeNonBackgroundFeatures method
	/** Mutex for the savedFeatures variable*/
	std::timed_mutex savedFeaturesMutex;

	/** Maximum pixel distance between two features to consider them equivalent */
	double featureEquivalencePositionThreshold_pixels = 1;
	/** Maximum angular distance between the observer directions for two features
	    to consider the two features equivalent */
	double featureEquivalenceAngularDistanceThreshold_radiants = 3.14159/8;//22.5 degrees
	/** Maximum difference between the observer distance for two features to consider
	    the two features equivalent.
			TODO: maybe this should not be a constant threshold, as a difference from
			      0.1 meters to 1.1 is very differnet form a difference from 10 meters to 11.
						Maybe a ratio? */
	double featureEquivalenceObserverDistanceThreshold_meters = 1;
	/** Maximum distance between the descriptors of two features to consider the two
	    features equivalent */
	unsigned int equivalentDescriptorsMaximumDistance = 20;
	/** Threshold for determining if the depth of two fetures is the same. In millimiters	 */
	unsigned int featureDepthMatchThreshold_mm = 50;
public:
	void saveFeature(const FeaturesMemory::Feature& newFeature);
	void saveFeatures(const std::vector<FeaturesMemory::Feature>& newFeatures);

	void removeNonBackgroundFeatures(const cv::Mat& depthImage);


	const std::vector<FeaturesMemory::Feature> getFeatures();
};


#endif

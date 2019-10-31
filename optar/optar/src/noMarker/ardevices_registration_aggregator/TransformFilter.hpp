#ifndef TRANSFORM_FILTER_HPP_20190320
#define TRANSFORM_FILTER_HPP_20190320

#include <tf/tf.h>

/**
 * Base virtual class for filters for transformations
 */
class TransformFilter
{
public:
	/**
	 * Updates the filter
	 * @param  pose New raw pose
	 * @return      The new pose estimate
	 */
	virtual tf::Pose update(const tf::Pose& pose) = 0;
};

#endif

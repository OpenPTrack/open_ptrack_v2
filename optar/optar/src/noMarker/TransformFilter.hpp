#ifndef TRANSFORM_FILTER_HPP_20190320
#define TRANSFORM_FILTER_HPP_20190320

#include <tf/tf.h>

class TransformFilter
{
public:	
	virtual tf::Pose update(const tf::Pose& pose) = 0;
};

#endif
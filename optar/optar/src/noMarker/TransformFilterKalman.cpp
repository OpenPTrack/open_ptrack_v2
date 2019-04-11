/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Methods implementations for the TransformFilterKalman class
 */

#include "TransformFilterKalman.hpp"
#include "../utils.hpp"


/**
 * Construct the kalman filter object, specifying the covariance values to use
 * @param processNoiseCovariance           Process noise covariance, models the variability of the actual real process value
 * @param measurementNoiseCovariance       Measurement noise covariance, models the amount of noise in the measurements
 * @param posterioriErrorCovariance        Initial value for the posteriori error covariance, which will be then actually estimated by the filter
 * @param startupFramesNum                 Number of input poses used to initialze the Kalman filter
 * @param estimateDistanceThreshold_meters Maximum distance between a new input pose and the current estimate for the new pose to be taken into consideration
 */
TransformFilterKalman::TransformFilterKalman(double processNoiseCovariance, double measurementNoiseCovariance, double posterioriErrorCovariance, int startupFramesNum, double estimateDistanceThreshold_meters) :
	mProcessNoiseCovariance(processNoiseCovariance),
	mMeasurementNoiseCovariance(measurementNoiseCovariance),
	mPosterioriErrorCovariance(posterioriErrorCovariance)
{
	kalmanFilter = cv::KalmanFilter();
	kalmanFilter.init(numberOfStates, numberOfMeasurements, numberOfInputs, CV_64F);                 // init Kalman Filter

	cv::setIdentity(kalmanFilter.processNoiseCov, cv::Scalar(mProcessNoiseCovariance));       // set process noise
	cv::setIdentity(kalmanFilter.measurementNoiseCov, cv::Scalar(mMeasurementNoiseCovariance));   // set measurement noise
	cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar(mPosterioriErrorCovariance));             // error covariance

	// DYNAMIC MODEL (matrix A):
	//
	//  [1 0 0 0 0 0]
	//  [0 1 0 0 0 0]
	//  [0 0 1 0 0 0]
	//  [0 0 0 1 0 0]
	//  [0 0 0 0 1 0]
	//  [0 0 0 0 0 1]

	cv::setIdentity(kalmanFilter.transitionMatrix, cv::Scalar(1));

	// MEASUREMENT MODEL (matrix H):
	//  [1 0 0 0 0 0]
	//  [0 1 0 0 0 0]
	//  [0 0 1 0 0 0]
	//  [0 0 0 1 0 0]
	//  [0 0 0 0 1 0]
	//  [0 0 0 0 0 1]
	cv::setIdentity(kalmanFilter.measurementMatrix, cv::Scalar(1));

	setupParameters(startupFramesNum, estimateDistanceThreshold_meters);
}


/**
 * Sets up the filter parameters
 * @param startupFramesNum                  number of frames used to initialize the filter before starting to use Kalman
 * @param estimateDistanceThreshold_meters  Maximum distance between a new input pose and the current estimate for the new pose to be taken into consideration
 */
void TransformFilterKalman::setupParameters(int startupFramesNum, double estimateDistanceThreshold_meters)
{
	this->startupFramesNum = startupFramesNum;
	this->estimateDistanceThreshold_meters = estimateDistanceThreshold_meters;
}


/**
 * Updates the kalman filter with a new pose measurement
 * @param pose The new pose
 *
 * @return the new kalman estimate
 */
tf::Pose TransformFilterKalman::updateKalman(const tf::Pose& pose)
{

	cv::Mat measurement = poseToMat(pose);

	if(!didKalmanEverUpdate)
	{
		didKalmanEverUpdate = true;
		kalmanFilter.statePost = measurement;
		return pose;
	}
	// First predict, to update the internal statePre variable
	cv::Mat prediction = kalmanFilter.predict();

	cv::Mat filtered = kalmanFilter.correct(measurement);


	return matToPose(kalmanFilter.statePost);
}


/**
 * Updates the filter with a new esimate. This update method doesn't just use the
 * kalman filter:
 *  - it uses the first few poses to compute an average with which it initailizes
 *    the Kalman filter
 *  - it drops poses that are too far from the current estimate
 *  - it detects jumps of the pose value, if a jump is detected the Kalman filter
 *    is forced to the new value
 * @param  pose The new raw pose
 * @return      The new estimate (i.e. filtered pose)
 */
tf::Pose TransformFilterKalman::update(const tf::Pose& pose)
{

	tf::Pose arcoreWorldFiltered;
	ROS_INFO_STREAM("Updating filter with raw transform "<<poseToString(pose));
	//if this is one of the first few frames
	if(rawEstimationsHistory.size()<=startupFramesNum)
	{
		rawEstimationsHistory.push_back(pose);
		//for the first few frame just use the average
		tf::Vector3 averagePosition = averagePosePositions(rawEstimationsHistory);

		//if the next frame uses the kalman filter then initialize it with the estimate that is closest to the average

		tf::Pose closestEstimate;
		double minimumDistance = std::numeric_limits<double>::max();
		for(tf::Pose estimate : rawEstimationsHistory)
		{
			double distance = averagePosition.distance(estimate.getOrigin());
			if(distance<minimumDistance)
			{
				minimumDistance=distance;
				closestEstimate = estimate;
			}
		}

		if(rawEstimationsHistory.size()==startupFramesNum)
			updateKalman(closestEstimate);

		arcoreWorldFiltered = closestEstimate;
	}
	else
	{
		if(poseDistance(pose,lastEstimate)>estimateDistanceThreshold_meters)
		{
			ROS_WARN_STREAM("New estimation is too different, discarding frame");
			return lastEstimate;
		}
		//if we are after the forst few frames use kalman
		arcoreWorldFiltered = updateKalman(pose);


		transformJumpDetectorHistory.push_back(pose);
		if(transformJumpDetectorHistory.size()>transformJumpDetectorLength)
			transformJumpDetectorHistory.pop_front();
		if(transformJumpDetectorHistory.size()>=transformJumpDetectorLength)
		{
			detectAndFollowTransformJump();
		}

		ROS_DEBUG_STREAM("filtering correction = "<<poseToString(pose*arcoreWorldFiltered.inverse()));
		ROS_INFO_STREAM("arcore_world_filtered = "<<poseToString(arcoreWorldFiltered));
	}

	lastEstimate = arcoreWorldFiltered;
	return lastEstimate;
}

/**
 * Forces the filter estimate to the provided pose
 * @param pose The new pose
 */
void TransformFilterKalman::forceState(const tf::Pose& pose)
{
	kalmanFilter.statePost = poseToMat(pose);
}

/**
 * Computes the variance between the distances of the provided poses.
 * The @p firstIndex and @p endIndex parameters allow to use a subset of the provided
 *  list instead of the while list
 * @param  transforms   The list of poses from which to get the distances
 * @param  firstIndex   Use this if you want the method to only use a subset of
 *                      the provided list. This indicates the beginning (included)
 *                      of the subset.
 * @param  endIndex     Use this if you want the method to only use a subset of
 *                      the provided list. This indicates the end (excluded) of
 *                      the subset.
 * @param  positionMean The mean distance between the poses in the provided set
 * @return              The variance of the distances
 */
double TransformFilterKalman::getDistanceVariance(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex, const tf::Vector3& positionMean)
{
	double variance = 0;
	ROS_INFO_STREAM("mean is "<<positionMean.x()<<"\t"<<positionMean.y()<<"\t"<<positionMean.z());
	int size = endIndex - firstIndex;
	for(int i=firstIndex;i<firstIndex+size;i++)
	{
		tf::Vector3 position = transforms.at(i).getOrigin();
		double dist = position.distance(positionMean);
		ROS_INFO_STREAM("element["<<i<<"] deviation = "<<dist);
		variance += dist*dist;
	}
	variance /= size;
	return variance;
}

/**
 * Computes the mean position of the provided poses
 * @param  transforms The poses
 * @param  firstIndex Use this if you want the method to only use a subset of
 *                    the provided list. This indicates the beginning (included)
 *                    of the subset.
 * @param  endIndex   Use this if you want the method to only use a subset of
 *                    the provided list. This indicates the end (excluded) of the
 *                    subset.
 * @return            [description]
 */
tf::Vector3 TransformFilterKalman::computePositionMean(std::deque<tf::Transform>& transforms, int firstIndex, int endIndex)
{
	tf::Vector3 positionMean(0,0,0);
	int size = endIndex - firstIndex;

	for(int i=firstIndex;i<firstIndex+size;i++)
	{
		tf::Vector3 position = transforms.at(i).getOrigin();
		positionMean.setX(positionMean.x() + position.x()/size);
		positionMean.setY(positionMean.y() + position.y()/size);
		positionMean.setZ(positionMean.z() + position.z()/size);
	}

	return positionMean;
}


/**
 * Detects a jump in the position of the estimated pose. If a jump is detected it
 * forces the estimate to a new estimated new value
 * @return true if it detected a jump, false if not
 */
bool TransformFilterKalman::detectAndFollowTransformJump()
{
	int historySize = transformJumpDetectorHistory.size();

	//compute variance using the kalman estimate as the mean
	double varianceOnKalman = getDistanceVariance(transformJumpDetectorHistory, 0, historySize, matToPose(kalmanFilter.statePost).getOrigin());

	//compute variance using the average of the poses in the history as the mean
	tf::Vector3 lastFewEstimatesPositionMean = computePositionMean(transformJumpDetectorHistory,0,historySize);
	double varianceOnHistoryAverage = getDistanceVariance(transformJumpDetectorHistory, 0, historySize,lastFewEstimatesPositionMean);

	ROS_INFO_STREAM(std::fixed << std::setw( 9 ) << std::setprecision( 7 )
				  <<"\nvarianceOnKalman          = "<<varianceOnKalman
		          <<"\nvarianceOnHistoryAverage  = "<<varianceOnHistoryAverage);
	if(varianceOnKalman > 2*varianceOnHistoryAverage)
	{
		ROS_WARN("Transform jumped! forcing estimation update");
		//it jumped

		//force the state to the first after the change
		forceState(transformJumpDetectorHistory.back());
		for(size_t i=0;i<transformJumpDetectorHistory.size();i++)
		{
			cv::Mat prediction = kalmanFilter.predict();
			cv::Mat estimated = kalmanFilter.correct(poseToMat(transformJumpDetectorHistory.at(i)));
		}
		/*
		//clean the history
		for(int i=0;i<historySize/2;i++)
		{
			transformJumpDetectorHistory.pop_front();
		}
		*/
		return true;
	}
	else
	{
		return false;
	}

}

/**
 * Converts a pose expressed as an OpenCV Mat to a tf Pose
 * @param  state The cv Mat
 * @return       The tf pose
 */
tf::Pose TransformFilterKalman::matToPose(const cv::Mat& state)
{

	tfScalar newRoll = state.at<double>(3);
	tfScalar newPitch = state.at<double>(4);
	tfScalar newYaw = state.at<double>(5);

	//ROS_INFO_STREAM("updated orientation (RPY) = "<<newRoll<<" \t"<<newPitch<<" \t"<<newYaw);
	tf::Quaternion newRotation;
	newRotation.setRPY(newRoll,newPitch,newYaw);
	return tf::Pose(newRotation,tf::Vector3(state.at<double>(0),state.at<double>(1),state.at<double>(2)));
}


/**
 * Converts a tf Pose to a pose expressed as an OpenCV Mat
 * @param  state The tf pose
 * @return       The OpenCV Mat
 */
cv::Mat TransformFilterKalman::poseToMat(const tf::Pose& pose)
{
	cv::Mat poseMat(6, 1, CV_64F);
	poseMat.at<double>(0) = pose.getOrigin().x();
	poseMat.at<double>(1) = pose.getOrigin().y();
	poseMat.at<double>(2) = pose.getOrigin().z();

	tfScalar roll,pitch,yaw;
	pose.getBasis().getRPY(roll,pitch,yaw);
	poseMat.at<double>(3) = roll;
	poseMat.at<double>(4) = pitch;
	poseMat.at<double>(5) = yaw;
	return poseMat;
}

/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Implementation of the TransformFilterKalman class, which implements a kalman filter for
 * 3D static poses
 */

#include "TransformFilterKalman.hpp"


/**
 * Construct the kalman filter object, specifying the covariance values to use
 * @param processNoiseCovariance Process noise covariance, models the variability of the actual real process value
 * @param measurementNoiseCovariance Measurement noise covariance, models the amount of noise in the measurements
 * @param posterioriErrorCovariance Initial value for the posteriori error covariance, which will be then actually estimated by the filter
 */
TransformFilterKalman::TransformFilterKalman(double processNoiseCovariance, double measurementNoiseCovariance, double posterioriErrorCovariance) : 
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

}


/**
 * Updates the estimation with a new pose measurement
 * @param pose The new pose
 *
 * @return the new estimate
 */
tf::Pose TransformFilterKalman::update(const tf::Pose& pose)
{	

	cv::Mat measurement = poseToMat(pose);

	if(!didEverUpdate)
	{
		didEverUpdate = true;
		kalmanFilter.statePost = measurement;
		return pose;
	}
	// First predict, to update the internal statePre variable
	cv::Mat prediction = kalmanFilter.predict();

	cv::Mat filtered = kalmanFilter.correct(measurement);



	transformJumpDetectorHistory.push_back(pose);
	if(transformJumpDetectorHistory.size()>transformJumpDetectorLength)
		transformJumpDetectorHistory.pop_front();
	if(transformJumpDetectorHistory.size()>=transformJumpDetectorLength)
	{
		detectAndFollowTransformJump();
	}

	return matToPose(kalmanFilter.statePost);
}



/**
 * Forces the filter to this state
 * @pram pose The new state
 *
 */
void TransformFilterKalman::forceState(const tf::Pose& pose)
{
	kalmanFilter.statePost = poseToMat(pose);	
}


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
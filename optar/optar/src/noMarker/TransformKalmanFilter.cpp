/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * Implementation of the TransformKalmanFilter class, which implements a kalman filter for
 * 3D static poses
 */

#include "TransformKalmanFilter.hpp"


/**
 * Construct the kalman filter object, specifying the covariance values to use
 * @param processNoiseCovariance Process noise covariance, models the variability of the actual real process value
 * @param measurementNoiseCovariance Measurement noise covariance, models the amount of noise in the measurements
 * @param posterioriErrorCovariance Initial value for the posteriori error covariance, which will be then actually estimated by the filter
 */
TransformKalmanFilter::TransformKalmanFilter(double processNoiseCovariance, double measurementNoiseCovariance, double posterioriErrorCovariance) : 
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
 * @pram pose The new pose
 *
 * @return the new estimate
 */
tf::Pose TransformKalmanFilter::update(tf::Pose pose)
{	

	cv::Mat measurement(6, 1, CV_64F);
	measurement.at<double>(0) = pose.getOrigin().x();
	measurement.at<double>(1) = pose.getOrigin().y();
	measurement.at<double>(2) = pose.getOrigin().z();

	tfScalar roll,pitch,yaw;
	pose.getBasis().getRPY(roll,pitch,yaw);
	measurement.at<double>(3) = roll;
	measurement.at<double>(4) = pitch;
	measurement.at<double>(5) = yaw;

	if(!didEverUpdate)
	{
		didEverUpdate = true;
		kalmanFilter.statePost = measurement;
	}
	// First predict, to update the internal statePre variable
	cv::Mat prediction = kalmanFilter.predict();

	cv::Mat estimated = kalmanFilter.correct(measurement);

	tfScalar newRoll = estimated.at<double>(3);
	tfScalar newPitch = estimated.at<double>(4);
	tfScalar newYaw = estimated.at<double>(5);

	//ROS_INFO_STREAM("updated orientation (RPY) = "<<newRoll<<" \t"<<newPitch<<" \t"<<newYaw);
	tf::Quaternion newRotation;
	newRotation.setRPY(newRoll,newPitch,newYaw);
	return tf::Pose(newRotation,tf::Vector3(estimated.at<double>(0),estimated.at<double>(1),estimated.at<double>(2)));
}
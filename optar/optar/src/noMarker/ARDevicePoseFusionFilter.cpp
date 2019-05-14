#include "ARDevicePoseFusionFilter.hpp"
#include <Eigen/Dense>

using namespace cv;
using namespace std;

/**
 * Initializes the filter
 */
ARDevicePoseFusionFilter::ARDevicePoseFusionFilter()
{
  //initialize the opencv kalman filter
  kalmanFilter = cv::KalmanFilter();
	kalmanFilter.init(stateVectorSize, measurementVectorSize, 0, CV_64F);

  double positionProcessCovariance = 	0.1;
  double speedProcessCovariance = 0.1;
  double accelerationProcessCovariance = 0.1;
  double quaternionProcessCovariance = 0;
  double angularVelocityProcessCovariance = 1;
  cv::setIdentity(kalmanFilter.processNoiseCov);// set process noise
  kalmanFilter.processNoiseCov.at<double>(0 ,0)  = positionProcessCovariance;//positions
  kalmanFilter.processNoiseCov.at<double>(1 ,1)  = positionProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(2 ,2)  = positionProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(3 ,3)  = speedProcessCovariance;//velocities
  kalmanFilter.processNoiseCov.at<double>(4 ,4)  = speedProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(5 ,5)  = speedProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(6 ,6)  = accelerationProcessCovariance;//accelerations
  kalmanFilter.processNoiseCov.at<double>(7 ,7)  = accelerationProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(8 ,8)  = accelerationProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(9 ,9)  = angularVelocityProcessCovariance;//angular velocities
  kalmanFilter.processNoiseCov.at<double>(10,10) = angularVelocityProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(11,11) = angularVelocityProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(12,12) = quaternionProcessCovariance;//quaternion error comes completely from the velocities
  kalmanFilter.processNoiseCov.at<double>(13,13) = quaternionProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(14,14) = quaternionProcessCovariance;
  kalmanFilter.processNoiseCov.at<double>(15,15) = quaternionProcessCovariance;

  double positionCovariance_arcore = 0.1;
  double quaternionCovariance_arcore = 0.1;
	cv::setIdentity(arcoreMeasurementNoiseCov);   // set measurement noise
  arcoreMeasurementNoiseCov.at<double>(0 ,0)  = positionCovariance_arcore;//positions
  arcoreMeasurementNoiseCov.at<double>(1 ,1)  = positionCovariance_arcore;
  arcoreMeasurementNoiseCov.at<double>(2 ,2)  = positionCovariance_arcore;
  arcoreMeasurementNoiseCov.at<double>(3 ,3)  = quaternionCovariance_arcore;//quaternion error comes completely from the velocities
  arcoreMeasurementNoiseCov.at<double>(4,4) = quaternionCovariance_arcore;
  arcoreMeasurementNoiseCov.at<double>(5,5) = quaternionCovariance_arcore;
  arcoreMeasurementNoiseCov.at<double>(6,6) = quaternionCovariance_arcore;

  double positionCovariance_pnp = 0.01;
  double quaternionCovariance_pnp = 0.01;
	cv::setIdentity(pnpMeasurementNoiseCov);   // set measurement noise
  pnpMeasurementNoiseCov.at<double>(0 ,0)  = positionCovariance_pnp;//positions
  pnpMeasurementNoiseCov.at<double>(1 ,1)  = positionCovariance_pnp;
  pnpMeasurementNoiseCov.at<double>(2 ,2)  = positionCovariance_pnp;
  pnpMeasurementNoiseCov.at<double>(3 ,3)  = quaternionCovariance_pnp;//quaternion error comes completely from the velocities
  pnpMeasurementNoiseCov.at<double>(4,4) = quaternionCovariance_pnp;
  pnpMeasurementNoiseCov.at<double>(5,5) = quaternionCovariance_pnp;
  pnpMeasurementNoiseCov.at<double>(6,6) = quaternionCovariance_pnp;

	cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar(1));// initialize to identity it will be adjusted by the filter
	cv::setIdentity(kalmanFilter.transitionMatrix, cv::Scalar(1));// will be changed when used
	cv::setIdentity(kalmanFilter.measurementMatrix, cv::Scalar(1));// will be changed when used
}

/**
 * Prediction function (i.e. f(x,u,0)).
 * @param  previousState the previous state
 * @param  timestep      time between the prediceted state and the previous state
 * @return               the predicted state ((x^-)_k)
 */
cv::Mat predictState(const cv::Mat& previousState, double timestep)
{
  // The differential equation controlling the orientation part is dq/dt = 1/2q*w
  // where w is the angular velocity vector (quaternion with w=0 and the angular
  // speeds on the imaginary components).
  // We "integrate" the function with euler. So we just multiply by the time
  // A more precise way would be to use 4th order Runge-Kutta
  double w1p = previousState.at<double>(9);
  double w2p = previousState.at<double>(10);
  double w3p = previousState.at<double>(11);


  //see "Orientation tracking for Humans and Robots Using Inertial Sensors" by BAchmann, Duman, ... , 1999
  Eigen::Quaternion prevOrientation(previousState.at<double>(12),
                                    previousState.at<double>(13),
                                    previousState.at<double>(14),
                                    previousState.at<double>(15));

  Eigen::Quaternion rateQuaternion = 0.5*prevOrientation*Eigen::Quaternion(0,w1p,w2p,w3p);
}

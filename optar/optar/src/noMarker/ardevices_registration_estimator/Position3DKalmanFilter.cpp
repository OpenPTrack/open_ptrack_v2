#include "Position3DKalmanFilter.hpp"

using namespace cv;
using namespace std;

/**
 * Constructs a new filter.
 * The measurement noise variance is set to 1 by default
 * The processNoiseVarianceFactor is set to 1 by default
 */
Position3DKalmanFilter::Position3DKalmanFilter()
{
  //initialize the opencv kalman filter
  kalmanFilter = cv::KalmanFilter();
  kalmanFilter.init(stateVectorSize, measurementVectorSize, 0, CV_64F);

  setupParameters(1, 1);
  cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar(0.1));

  kalmanFilter.measurementMatrix = (Mat_<double>(3,9) << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 1, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 0, 1, 0, 0, 0, 0, 0, 0);
}

/**
 * Sets up the parameters for the filtering
 * @param measurementNoiseVariance Variance for the measurement noise, will be applied to all of x, y and z
 * @param processNoiseVarianceFactor Multiplicative factor using in the process noise covariance matrix computation
 */
void Position3DKalmanFilter::setupParameters( double measurementNoiseVariance, double processNoiseVarianceFactor)
{
  this->processNoiseVarianceFactor = processNoiseVarianceFactor;
  double var = measurementNoiseVariance;
  kalmanFilter.measurementNoiseCov = (Mat_<double>(3,3) <<  var, 0,   0,
                                                            0,   var, 0,
                                                            0,   0,   var);
}

/**
 * Computes the transition matrix (A) for the provided timestep
 * @param  timestep The time since the last update, in seconds
 * @return          The transition matrix
 */
cv::Mat Position3DKalmanFilter::transitionMatrix(double timestep)
{
  double t = timestep;
  cv::Mat A = (Mat_<double>(9,9) << 1, 0, 0, t, 0, 0, t*t/2, 0,     0,
                                    0, 1, 0, 0, t, 0, 0,     t*t/2, 0,
                                    0, 0, 1, 0, 0, t, 0,     0,     t*t/2,
                                    0, 0, 0, 1, 0, 0, t,     0,     0,
                                    0, 0, 0, 0, 1, 0, 0,     t,     0,
                                    0, 0, 0, 0, 0, 1, 0,     0,     t,
                                    0, 0, 0, 0, 0, 0, 1,     0,     0,
                                    0, 0, 0, 0, 0, 0, 0,     1,     0,
                                    0, 0, 0, 0, 0, 0, 0,     0,     1);
  return A;
}

/**
 * Computes the process noise covariance matrix using the specified timestep
 * @param  timestep The time since the last update, in seconds
 * @return          The process noise matrix
 */
cv::Mat Position3DKalmanFilter::processNoiseCovariance(double timestep)
{
  double t = timestep;
  cv::Mat G = (Mat_<double>(9,3) << t*t/2,  0,      0,
                                    t,      0,      0,
                                    1,      0,      0,
                                    0,      t*t/2,  0,
                                    0,      t,      0,
                                    0,      1,      0,
                                    0,      0,      t*t/2,
                                    0,      0,      t,
                                    0,      0,      1);
  cv::Mat Q = G*G.t()*processNoiseVarianceFactor;
  return Q;
}

/**
 * Updates the state by predictiong and correcting
 * @param  measurement   The new measurement
 * @param  timestep      The time since the last update, in seconds
 * @return               The updated state
 */
cv::Mat Position3DKalmanFilter::update(const cv::Mat& measurement, double timestep_sec)
{
  kalmanFilter.transitionMatrix = transitionMatrix(timestep_sec);
  kalmanFilter.processNoiseCov  = processNoiseCovariance(timestep_sec);
  if(didEverComputeState)
  {
    //measurement matrix is ok
    //measurement noise matrix should be ok
    //ROS_INFO("Predicting");
    //ROS_INFO_STREAM("temp1.size()="<<kalmanFilter.temp1.size().height<<";"<<kalmanFilter.temp1.size().width);
    //ROS_INFO_STREAM("transitionMatrix.size()="<<kalmanFilter.transitionMatrix.size().height<<";"<<kalmanFilter.transitionMatrix.size().width);
    //ROS_INFO_STREAM("processNoiseCov.size()="<<kalmanFilter.processNoiseCov.size().height<<";"<<kalmanFilter.processNoiseCov.size().width);
    kalmanFilter.predict();
    //ROS_INFO("Correcting");
    return kalmanFilter.correct( measurement);
  }
  else
  {
    //ROS_INFO("Initializing Kalman filter");
    kalmanFilter.statePost = (Mat_<double>(9,1) <<  measurement.at<double>(0,0),
                                                    measurement.at<double>(1,0),
                                                    measurement.at<double>(2,0),
                                                    0,
                                                    0,
                                                    0,
                                                    0,
                                                    0,
                                                    0);
    didEverComputeState = true;
    return measurement;
  }
}

/**
 * Updates the state by predictiong and correcting
 * @param  measurement   The new measurement
 * @param  timestep      The time since the last update, in seconds
 * @return               The updated state
 */
tf::Vector3 Position3DKalmanFilter::update(const tf::Vector3& measurement, double timestep_sec)
{
  cv::Mat measurement_cv = (Mat_<double>(3,1) << measurement.x(), measurement.y(), measurement.z());
  cv::Mat newState = update(measurement_cv, timestep_sec);
  return tf::Vector3(newState.at<double>(0,0),newState.at<double>(1,0),newState.at<double>(2,0));
}

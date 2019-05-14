#include "Position3DKalmanFilter.hpp"

using namespace cv;
using namespace std;

Position3DKalmanFilter::Position3DKalmanFilter()
{
  //initialize the opencv kalman filter
  kalmanFilter = cv::KalmanFilter();
  kalmanFilter.init(stateVectorSize, measurementVectorSize, 0, CV_64F);

  setupParameters(0.1,0.1,0.1);
  cv::setIdentity(kalmanFilter.errorCovPost, cv::Scalar(0.1));

  kalmanFilter.measurementMatrix = (Mat_<double>(3,9) << 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 1, 0, 0, 0, 0, 0, 0, 0,
                                                         0, 0, 1, 0, 0, 0, 0, 0, 0);
}

void Position3DKalmanFilter::setupParameters( double measurementNoiseVarianceX,
                                              double measurementNoiseVarianceY,
                                              double measurementNoiseVarianceZ)
{
  double varX = measurementNoiseVarianceX;
  double varY = measurementNoiseVarianceY;
  double varZ = measurementNoiseVarianceZ;
  kalmanFilter.measurementNoiseCov = (Mat_<double>(3,3) <<  varX, 0,    0,
                                                            0,    varY, 0,
                                                            0,    0,    varZ);
}

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


cv::Mat Position3DKalmanFilter::processNoiseCovariance(double timestep)
{
  double t = timestep;
  cv::Mat G = (Mat_<double>(3,9) << t*t/2,  0,      0,
                                    t,      0,      0,
                                    1,      0,      0,
                                    0,      t*t/2,  0,
                                    0,      t,      0,
                                    0,      1,      0,
                                    0,      0,      t*t/2,
                                    0,      0,      t,
                                    0,      0,      1);
  double sigma = 1;
  cv::Mat Q = G*G.t()*sigma;
  return Q;
}

/**
 * Updates the state by predictiong and correcting
 * @param  measurement   The new measurement
 * @param  timestep      The time since the last update, in seconds
 * @return               The updated state
 */
cv::Mat Position3DKalmanFilter::update(cv::Mat measurement, double timestep_sec)
{
  kalmanFilter.transitionMatrix = transitionMatrix(timestep_sec);
  kalmanFilter.processNoiseCov  = processNoiseCovariance(timestep_sec);
  //measurement matrix is ok
  //measurement noise matrix should be ok
  kalmanFilter.predict();
  return kalmanFilter.correct( measurement);
}

tf::Vector3 Position3DKalmanFilter::update(tf::Vector3 measurement, double timestep_sec)
{
  cv::Mat measurement_cv = (Mat_<double>(3,1) << measurement.x(), measurement.y(), measurement.z());
  cv::Mat newState = update(measurement_cv, timestep_sec);
  return tf::Vector3(newState.at<double>(0,0),newState.at<double>(1,0),newState.at<double>(2,0));
}

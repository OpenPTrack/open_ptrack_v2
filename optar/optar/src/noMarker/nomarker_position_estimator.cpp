/**
 * @file
 *
 * @author Carlo Rizzardo
 *
 * This file implements a ros node which estimates the transformation between an ARCore 
 * coordinae frame and the ros tf /world frame
 */


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xphoto.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>         // std::this_thread::sleep_for
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>


#include <dynamic_reconfigure/server.h>
#include <optar/OptarDynamicParametersConfig.h>

#include "../utils.hpp"
#include  "TransformKalmanFilter.hpp"


using namespace cv;
using namespace std;


const string NODE_NAME 						= "nomarker_position_estimator";
const string input_arcore_topic				= "arcore_camera";
const string input_kinect_camera_topic		= "kinect_camera" ;
const string input_kinect_depth_topic		= "kinect_depth";
const string input_kinect_camera_info_topic	= "kinect_camera_info" ;


const string output_pose_raw_topic			= "optar/no_marker_pose_raw";
const string output_pose_marker_topic		= "optar/pose_marker";


geometry_msgs::TransformStamped transformKinectToWorld;
ros::Publisher pose_raw_pub;
ros::Publisher pose_marker_pub;


class PoseEstimation
{
public:
	std::vector<tf::Pose> arcoreWorldHistory;
	tf::Pose lastEstimate;
};


std::map<std::string, PoseTransformEstimator> phoneEstimations;

//These may be overwritten by dynamic_reconfigure
static double pnpReprojectionError = 5;
static double pnpConfidence = 0.99;
static double pnpIterations = 1000;
static double matchingThreshold = 25;
static double reprojectionErrorDiscardThreshold = 5;
static int orbMaxPoints = 500;
static double orbScaleFactor = 1.2;
static int orbLevelsNumber = 8;
static unsigned int startupFramesNum = 10;
static double phoneOrientationDifferenceThreshold_deg = 45;
static double estimateDistanceThreshold_meters = 5;

static bool showImages = true;
static shared_ptr<TransformKalmanFilter> transformKalmanFilter;




void dynamicParametersCallback(optar::OptarDynamicParametersConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure Request: %f %f %f", 
            config.pnp_iterations,
            config.pnp_confidence, 
            config.pnp_reprojection_error);

	pnpReprojectionError = config.pnp_reprojection_error;
	pnpConfidence 					= config.pnp_confidence;
	pnpIterations 					= config.pnp_iterations;

	matchingThreshold 				= config.matching_threshold;
	reprojectionErrorDiscardThreshold = config.reprojection_discard_threshold;

	orbMaxPoints		= config.orb_max_points;
	orbScaleFactor	= config.orb_scale_factor;
	orbLevelsNumber	= config.orb_levels_number;

	startupFramesNum	= config.startup_frames_num;
	phoneOrientationDifferenceThreshold_deg	= config.phone_orientation_diff_thresh;
}


int findOrbMatches(	const cv::Mat& arcoreImg, 
					const cv::Mat& kinectCameraImg, 
					std::vector<cv::DMatch>& matches, 
					std::vector<cv::KeyPoint>& arcoreKeypoints, 
					std::vector<cv::KeyPoint>& kinectKeypoints)
{
	matches.clear();
    cv::Ptr<cv::ORB> orb = cv::ORB::create(orbMaxPoints,orbScaleFactor,orbLevelsNumber);

    //detect keypoints on both images
   	std::chrono::steady_clock::time_point beforeDetection = std::chrono::steady_clock::now();
    orb->detect(kinectCameraImg, kinectKeypoints);
    orb->detect(arcoreImg, arcoreKeypoints);
   	std::chrono::steady_clock::time_point afterDetection = std::chrono::steady_clock::now();
    unsigned long detectionDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDetection - beforeDetection).count();
	ROS_INFO("Keypoints detection took %lu ms",detectionDuration);
	ROS_INFO_STREAM("Found "<<arcoreKeypoints.size()<<" keypoints for arcore, "<<kinectKeypoints.size()<<" for kinect");

	if ( kinectKeypoints.empty() )
	{
		ROS_ERROR("No keypoints found for kinect");
		return -1;
	}
	if ( arcoreKeypoints.empty() )
	{
		ROS_ERROR("No keypoints found for arcore");
		return -2;
	}

	//compute descriptors for the keypoints in both images
   	std::chrono::steady_clock::time_point beforeDescriptors = std::chrono::steady_clock::now();
    cv::Mat kinectDescriptors;
    cv::Mat arcoreDescriptors;
    orb->compute(kinectCameraImg,kinectKeypoints,kinectDescriptors);
	orb->compute(arcoreImg,arcoreKeypoints,arcoreDescriptors);
   	std::chrono::steady_clock::time_point aftereDescriptors = std::chrono::steady_clock::now();
	unsigned long descriptorsDuration = std::chrono::duration_cast<std::chrono::milliseconds>(aftereDescriptors - beforeDescriptors).count();
	ROS_INFO("Descriptors computation took %lu ms",descriptorsDuration);

	if ( kinectDescriptors.empty() )
	{
		ROS_ERROR("No descriptors for kinect");
		return -3;
	}
	if ( arcoreDescriptors.empty() )
	{
		ROS_ERROR("no descriptors for arcore");
		return -4;
	}
  	 
  	//find matches between the descriptors
   	std::chrono::steady_clock::time_point beforeMatching = std::chrono::steady_clock::now();
    cv::BFMatcher::create(cv::NORM_HAMMING)->match(arcoreDescriptors,kinectDescriptors,matches);//arcore is query, kinect is trains
   	std::chrono::steady_clock::time_point afterMatching = std::chrono::steady_clock::now();
	unsigned long matchingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterMatching - beforeMatching).count();
	ROS_INFO("Descriptors matching took %lu ms",matchingDuration);
	return 0;
}

int filterMatches(const std::vector<cv::DMatch>& matches, std::vector<cv::DMatch>& goodMatches)
{
	double max_dist = -10000000;
  	double min_dist = 10000000;
	//Find minimum and maximum distances between matches
	for( unsigned int i = 0; i < matches.size(); i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist )
		{
			min_dist = dist;
		}
		if( dist > max_dist )
			max_dist = dist;
	}
	ROS_INFO("Max match dist : %f", max_dist );
	ROS_INFO("Min match dist : %f", min_dist );

	
	//Filter the matches to keep just the best ones
	for( unsigned int i = 0; i < matches.size(); i++ )
	{
		if( matches[i].distance <= matchingThreshold/*std::min(min_dist+(max_dist-min_dist)*0.2,25.0)*/ )
		{
			goodMatches.push_back( matches[i]);
			//ROS_INFO_STREAM("got a good match, dist="<<matches[i].distance);
		}
	}

/*
	//take best 4
	std::vector<int> goodMatchesIdx;
	for(int i=0;i<4;i++)
	{
		double bestDist = 10000000;
		int bestMatchIdx = 0;
		for(unsigned int j=0;j<matches.size();j++)
		{
			double dist = matches[j].distance;
			if( dist < bestDist && std::find(goodMatchesIdx.begin(), goodMatchesIdx.end(), j) == goodMatchesIdx.end())//if it is better and it is not in goodMatches
			{
				bestDist = dist;
				bestMatchIdx = j;
			}
		}
		goodMatchesIdx.push_back(bestMatchIdx);
		goodMatches.push_back(matches.at(bestMatchIdx));
	}
	*/
	//goodMatches.push_back(*bestMatch);
	return 0;
}

int readReceivedMessages(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo,
					cv::Mat& arcoreCameraMatrix,
					cv::Mat& arcoreImg,
					cv::Mat& kinectCameraImg,
					cv::Mat& kinectDepthImg,
					tf::Pose& phonePoseArcoreFrameConverted)
{
	std::chrono::steady_clock::time_point beginning = std::chrono::steady_clock::now();

	//convert the camera matrix in a cv::Mat
	arcoreCameraMatrix = cv::Mat(3, 3, CV_64FC1);
	arcoreCameraMatrix.at<double>(0,0) = arcoreInputMsg->focal_length_x_px;
	arcoreCameraMatrix.at<double>(0,1) = 0;
	arcoreCameraMatrix.at<double>(0,2) = arcoreInputMsg->principal_point_x_px;
	arcoreCameraMatrix.at<double>(1,0) = 0;
	arcoreCameraMatrix.at<double>(1,1) = arcoreInputMsg->focal_length_y_px;
	arcoreCameraMatrix.at<double>(1,2) = arcoreInputMsg->principal_point_y_px;
	arcoreCameraMatrix.at<double>(2,0) = 0;
	arcoreCameraMatrix.at<double>(2,1) = 0;
	arcoreCameraMatrix.at<double>(2,2) = 1;

	//decode arcore image
	cv::Mat rawImageData(cv::Mat(arcoreInputMsg->image.data));
	if(rawImageData.empty())
	{
		ROS_ERROR("Invalid arcore image (it's empty!)");
		return -1;
	}

	arcoreImg = cv::imdecode(rawImageData,1);//convert compressed image data to cv::Mat
	if(!arcoreImg.data)
	{
		ROS_ERROR("couldn't decode arcore image");
		return -2;
	}
	if(arcoreImg.channels()!=3)
	{
		ROS_ERROR("Color image expected from arcore device, received something different");
		return -3;
	}
	//The image sent by the Android app is monochrome, but it is stored in a 3-channel PNG image as the red channel
	//So we extract the red channel and use that.
	//Also the image is flipped on the y axis
	cv::Mat planes[3];
	split(arcoreImg,planes);  // planes[2] is the red channel
	arcoreImg = planes[2];
	cv::Mat flippedArcoreImg;
	cv::flip(arcoreImg,flippedArcoreImg,0);
	arcoreImg=flippedArcoreImg;
	//cv::xphoto::createSimpleWB()->balanceWhite(flippedArcoreImg,arcoreImg);
	cv::equalizeHist(arcoreImg,arcoreImg);
    ROS_INFO("decoded arcore image");
    //cv::imshow("Arcore", arcoreImg);

    //decode kinect rgb image
	kinectCameraImg = cv_bridge::toCvShare(kinectInputCameraMsg)->image;//convert compressed image data to cv::Mat
	if(!kinectCameraImg.data)
	{
		ROS_ERROR("couldn't extract kinect camera opencv image");
		return -4;
	}
    ROS_INFO("decoded kinect camera image");
	cv::equalizeHist(kinectCameraImg,kinectCameraImg);
    //cv::imshow("Kinect", kinectCameraImg);


    //decode kinect depth image
	kinectDepthImg = cv_bridge::toCvShare(kinectInputDepthMsg)->image;//convert compressed image data to cv::Mat
	if(!kinectDepthImg.data)
	{
		ROS_ERROR("couldn't extract kinect depth opencv image");
		return -5;
	}
    ROS_INFO("decoded kinect depth image");
  /*  cv::namedWindow("KinectDepth", cv::WINDOW_NORMAL);
	cv::resizeWindow("KinectDepth",1280,720);
    cv::imshow("KinectDepth", kinectDepthImg);
*/






	// Convert phone arcore pose
	// ARCore on Unity uses Unity's coordinate systema, which is left-handed, normally in arcore for Android the arcore 
	// camera position is defined with x pointing right, y pointing up and -z pointing where the camera is facing.
	// As provided from all ARCore APIs, Poses always describe the transformation from object's local coordinate space 
	// to the world coordinate space. This is the usual pose representation, same as ROS
	tf::Pose phonePoseArcoreFrameUnity;
	tf::poseMsgToTF(arcoreInputMsg->mobileFramePose,phonePoseArcoreFrameUnity);
	tf::Pose phonePoseArcoreFrame = convertPoseUnityToRos(phonePoseArcoreFrameUnity);

	//publishTransformAsTfFrame(phonePoseArcoreFrame,"phone_arcore","/world",arcoreInputMsg->header.stamp);
	//publishTransformAsTfFrame(phonePoseArcoreFrameUnity,"phone_arcore_left","/world",arcoreInputMsg->header.stamp);

	//from x to the right, y up, z back to x to the right, y down, z forward
	tf::Transform cameraConventionTransform = tf::Transform(tf::Quaternion(tf::Vector3(1,0,0), 3.1415926535897));//rotate 180 degrees around x axis
	//assuming z is pointing foward:
	tf::Transform portraitToLandscape = tf::Transform(tf::Quaternion(tf::Vector3(0,0,1), 3.1415926535897/2));//rotate +90 degrees around z axis
	tf::Transform justRotation = tf::Transform(phonePoseArcoreFrame.getRotation()) * portraitToLandscape;
	tf::Transform justTranslation = tf::Transform(tf::Quaternion(1,0,0,0),phonePoseArcoreFrame.getOrigin());

	//tf::Pose phonePoseArcoreInverted = tf::Transform(tf::Quaternion(tf::Vector3(1,0,0),0),phonePoseArcoreFrame.getOrigin()).inverse() * tf::Transform(phonePoseArcoreFrame.getRotation()).inverse();
	phonePoseArcoreFrameConverted =  justTranslation *cameraConventionTransform*justRotation;
	ROS_INFO_STREAM("phonePoseArcoreFrame = "<<poseToString(phonePoseArcoreFrame));
	//publishTransformAsTfFrame(phonePoseArcoreFrameConverted,"phone_arcore_converted","/world",arcoreInputMsg->header.stamp);
	//publishTransformAsTfFrame(phonePoseArcoreInverted,"phone_arcore_inv","/world",arcoreInputMsg->header.stamp);


	std::chrono::steady_clock::time_point afterDecoding = std::chrono::steady_clock::now();
	unsigned long decodingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDecoding - beginning).count();
	ROS_INFO("Images decoding and initialization took %lu ms",decodingDuration);

	return 0;
}


/**
 * Checks the provided matches to ensure they have valid depth info. If the depth is not available in the depth image
 * this funciton will try to fix the image by getting the closest depth value. If the closest valid pixel is too far
 * the match will be dropped.
 */
int fixMatchesDepthOrDrop(const std::vector<cv::DMatch>& inputMatches, const std::vector<cv::KeyPoint>& kinectKeypoints, cv::Mat& kinectDepthImg,std::vector<cv::DMatch>& outputMatches)
{
	outputMatches.clear();
	for( unsigned int i = 0; i < inputMatches.size(); i++ )
	{
		//QueryIdx is for arcore descriptors, TrainIdx is for kinect. This is because of how we passed the arguments to BFMatcher::match
		cv::Point2f imgPos = kinectKeypoints.at(inputMatches.at(i).trainIdx).pt;
		//try to find the depth using the closest pixel
		if(kinectDepthImg.at<uint16_t>(imgPos)==0)
		{
			Point2i nnz = findNearestNonZeroPixel(kinectDepthImg,imgPos.x,imgPos.y,100);
			double nnzDist = hypot(nnz.x-imgPos.x,nnz.y-imgPos.y);
			nnz = findLowestNonZeroInRing(kinectDepthImg,imgPos.x,imgPos.y, nnzDist+10, nnzDist);

			//ROS_INFO("Got closest non-zero pixel, %d;%d",nnz.x,nnz.y);
			kinectDepthImg.at<uint16_t>(imgPos)=kinectDepthImg.at<uint16_t>(nnz);
		}

		if(kinectDepthImg.at<uint16_t>(imgPos)==0)
		{
			//ROS_INFO("Dropped match as it had zero depth");
		}
		else
		{
			outputMatches.push_back(inputMatches.at(i));
		}
	}
	return 0;
}


int get3dPositionsAndImagePositions(const std::vector<cv::DMatch>& inputMatches,
	const std::vector<cv::KeyPoint>& kinectKeypoints,
	const std::vector<cv::KeyPoint>& arcoreKeypoints,
	const cv::Mat& kinectDepthImg,
	const sensor_msgs::CameraInfo& kinectCameraInfo, 
    std::vector<cv::Point3f>& matches3dPos,
    std::vector<cv::Point2f>& matchesImgPos)
{
	matches3dPos.clear();
	matchesImgPos.clear();
	for( unsigned int i = 0; i < inputMatches.size(); i++ )
	{
		//QueryIdx is for arcore descriptors, TrainIdx is for kinect. This is because of how we passed the arguments to BFMatcher::match
		cv::Point2f kinectPixelPos	= kinectKeypoints.at(inputMatches.at(i).trainIdx).pt;
		cv::Point2f arcorePixelPos	= arcoreKeypoints.at(inputMatches.at(i).queryIdx).pt;
		cv::Point3f pos3d 			= get3dPoint(	kinectPixelPos.x,kinectPixelPos.y,
													kinectDepthImg.at<uint16_t>(kinectPixelPos),
													kinectCameraInfo.P[0+4*0],kinectCameraInfo.P[1+4*1],kinectCameraInfo.P[2+4*0],kinectCameraInfo.P[2+4*1]);
		matches3dPos.push_back(pos3d);
		matchesImgPos.push_back(arcorePixelPos);


		ROS_DEBUG_STREAM("good match between "<<kinectPixelPos.x<<";"<<kinectPixelPos.y<<" \tand \t"<<arcorePixelPos.x<<";"<<arcorePixelPos.y<<" \tdistance = "<<inputMatches.at(i).distance);
		//ROS_INFO_STREAM("depth = "<<kinectDepthImg.at<uint16_t>(kinectPixelPos));
	}
	return 0;
}

void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo)
{
	std::chrono::steady_clock::time_point beginning = std::chrono::steady_clock::now();
	long arcoreTime = arcoreInputMsg->header.stamp.sec*1000000000L + arcoreInputMsg->header.stamp.nsec;
	long kinectTime = kinectInputCameraMsg->header.stamp.sec*1000000000L + kinectInputCameraMsg->header.stamp.nsec;
	ROS_INFO("\n\n\n\n\n\nreceived images. time diff = %+7.5f sec.  arcore time = %012ld  kinect time = %012ld",(arcoreTime-kinectTime)/1000000000.0, arcoreTime, kinectTime);







	//:::::::::::::::Decode received images and stuff::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

	int r;



	cv::Mat arcoreCameraMatrix;
	cv::Mat arcoreImg;
	cv::Mat kinectCameraImg;
	cv::Mat kinectDepthImg;
	tf::Pose phonePoseArcoreFrameConverted;
	r = readReceivedMessages( arcoreInputMsg,
					kinectInputCameraMsg,
					kinectInputDepthMsg,
					kinectCameraInfo,
					arcoreCameraMatrix,
					arcoreImg,
					kinectCameraImg,
					kinectDepthImg,
					phonePoseArcoreFrameConverted);
	if(r<0)
	{
		ROS_ERROR("Invalid input messages. Dropping frame");
		return;
	}
	


	//:::::::::::::::Find the matches between the images::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::









	std::chrono::steady_clock::time_point beforeMatching = std::chrono::steady_clock::now();

	//find matches
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint>  arcoreKeypoints;
    std::vector<cv::KeyPoint>  kinectKeypoints;
    r = findOrbMatches(arcoreImg, kinectCameraImg, matches, arcoreKeypoints, kinectKeypoints);
    if(r<0)
    {
    	ROS_ERROR("error finding matches");
    	return;
    }
  	ROS_INFO_STREAM("got "<<matches.size()<<" matches");

    //filter matches
	std::vector< cv::DMatch > goodMatchesWithNull;
  	r = filterMatches(matches,goodMatchesWithNull);
  	if(r<0)
  	{
  		ROS_ERROR("error filtering matches");
  		return;
  	}
  	ROS_INFO("Got %lu good matches, but some could be invalid",goodMatchesWithNull.size());

  	//On the kinect side the depth could be zero at the match location
  	//we try to get the nearest non-zero depth, if it's too far we discard the match
	std::vector< cv::DMatch > goodMatches;
	r = fixMatchesDepthOrDrop(goodMatchesWithNull, kinectKeypoints, kinectDepthImg ,goodMatches);
  	if(r<0)
  	{
  		ROS_ERROR("error fixing matches depth");
  		return;
  	}
  	ROS_INFO_STREAM("got "<<goodMatches.size()<<" actually good matches");



	std::chrono::steady_clock::time_point afterMatchesComputation = std::chrono::steady_clock::now();
	unsigned long matchesComputationDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterMatchesComputation - beforeMatching).count();
	ROS_INFO("Matches computation took %lu ms",matchesComputationDuration);







	//:::::::::::::::Find the 3d position of the matches::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::












    //find the 3d poses corresponding to the goodMatches, these will be relative to the kinect frame
    std::vector<cv::Point3f> goodMatches3dPos;
    std::vector<cv::Point2f> goodMatchesImgPos;

	r = get3dPositionsAndImagePositions(goodMatches,kinectKeypoints, arcoreKeypoints, kinectDepthImg,kinectCameraInfo,goodMatches3dPos, goodMatchesImgPos);
	if(r<0)
	{
  		ROS_ERROR("error getting matching points");
  		return;		
	}

	
	

	//used to publish visualizations to rviz
	visualization_msgs::MarkerArray markerArray;
	for(unsigned int i=0; i<goodMatches3dPos.size(); i++)//build the rviz markers
	{	
		cv::Point3f pos3d = goodMatches3dPos.at(i);
		markerArray.markers.push_back( buildMarker(	pos3d,"match"+std::to_string(i),0,0,1,1, 0.2, kinectInputCameraMsg->header.frame_id));//matches are blue
	}
	pose_marker_pub.publish(markerArray);		
	cv::Mat matchesImg;
	if(showImages)
		cv::drawMatches(arcoreImg, arcoreKeypoints, kinectCameraImg, kinectKeypoints, goodMatches, matchesImg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	//If we have less than 4 matches we cannot procede
	if(goodMatches.size()<4)
	{
		//cv::drawKeypoints(arcoreImg,arcoreKeypoints)
		if(showImages)
		{
			prepareOpencvImageForShowing("Matches", matchesImg, matchesImg.rows * 1920/matchesImg.cols);
		    cv::waitKey(1);
		}
		ROS_WARN("not enough matches to determine position");
		return;
	}





	std::chrono::steady_clock::time_point after3dpositionsComputation = std::chrono::steady_clock::now();
	unsigned long _3dPositionsComputationDuration = std::chrono::duration_cast<std::chrono::milliseconds>(after3dpositionsComputation - afterMatchesComputation).count();
	ROS_INFO("3D positions computation took %lu ms",_3dPositionsComputationDuration);





	//:::::::::::::::Determine the phone position::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::










	
	ROS_INFO_STREAM("arcoreCameraMatrix = \n"<<arcoreCameraMatrix);
	cv::Mat tvec;
	cv::Mat rvec;
	std::vector<int> inliers;
	ROS_INFO_STREAM("Running pnpRansac with iterations="<<pnpIterations<<" pnpReprojectionError="<<pnpReprojectionError<<" pnpConfidence="<<pnpConfidence);
	cv::solvePnPRansac(	goodMatches3dPos,goodMatchesImgPos,
						arcoreCameraMatrix,cv::noArray(),
						rvec,tvec,
						false,
						pnpIterations,
						pnpReprojectionError,
						pnpConfidence,
						inliers);

	ROS_INFO_STREAM("solvePnPRansac used "<<inliers.size()<<" inliers and says:\t tvec = "<<tvec.t()<<"\t rvec = "<<rvec.t());

	std::chrono::steady_clock::time_point afterPnpComputation = std::chrono::steady_clock::now();
	unsigned long pnpComputationDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterPnpComputation - after3dpositionsComputation).count();
	ROS_INFO("PNP computation took %lu ms",pnpComputationDuration);

	//reproject points to then check reprojection error (and visualize them)
	std::vector<Point2f> reprojPoints;
	cv::projectPoints 	(goodMatches3dPos,
						rvec, tvec,
						arcoreCameraMatrix,
						cv::noArray(),
						reprojPoints);



	cv::Mat colorArcoreImg;
	if(showImages)
		cvtColor(arcoreImg, colorArcoreImg, CV_GRAY2RGB);
	double reprojError = 0;
	//calculate reprojection error mean and draw reprojections
	for(unsigned int i=0;i<inliers.size();i++)
	{
		Point2f pix = goodMatchesImgPos.at(inliers.at(i));
		Point2f reprojPix = reprojPoints.at(inliers.at(i));
		reprojError += hypot(pix.x-reprojPix.x, pix.y-reprojPix.y)/reprojPoints.size();

		if(showImages)
		{
			cv::circle(colorArcoreImg,pix,15,Scalar(128,128,128),5);
			int r = ((double)rand())/RAND_MAX*255;
			int g = ((double)rand())/RAND_MAX*255;
			int b = ((double)rand())/RAND_MAX*255;
			Scalar color = Scalar(r,g,b);
			cv::line(colorArcoreImg,pix,reprojPix,color,3);
		}
	}
	ROS_INFO_STREAM("inliers reprojection error = "<<reprojError);


	std::chrono::steady_clock::time_point reprojectionComputation = std::chrono::steady_clock::now();
	unsigned long reprojectionComputationDuration = std::chrono::duration_cast<std::chrono::milliseconds>(reprojectionComputation- afterPnpComputation).count();
	ROS_INFO("Reprojection error computation took %lu ms",reprojectionComputationDuration);

	if(showImages)
	{
		prepareOpencvImageForShowing("Matches", matchesImg, 800);
		prepareOpencvImageForShowing("Reprojection", colorArcoreImg, 400);
		cv::waitKey(1);
	}


	std::chrono::steady_clock::time_point afterDrawMatches = std::chrono::steady_clock::now();
	unsigned long drawMathcesDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDrawMatches - reprojectionComputation).count();
	ROS_INFO("draw matches took %lu ms",drawMathcesDuration);


	//discard bad frames
	if(reprojError>reprojectionErrorDiscardThreshold)
	{
		ROS_WARN("Reprojection error beyond threshold, discarding frame");
		return;
	}

	//convert to ros format
	Eigen::Vector3d position;
	Eigen::Quaterniond rotation;
	opencvPoseToEigenPose(rvec,tvec,position,rotation);
	geometry_msgs::Pose poseNotStamped = buildRosPose(position,rotation);
	//invert the pose, because that's what you do
	tf::Pose poseTf;
	tf::poseMsgToTF(poseNotStamped,poseTf);
	tf::poseTFToMsg(poseTf.inverse(),poseNotStamped);
	//make it stamped
	geometry_msgs::PoseStamped phonePoseKinect;
	phonePoseKinect.pose = poseNotStamped;
	phonePoseKinect.header.frame_id = "kinect01_rgb_optical_frame";
	phonePoseKinect.header.stamp = arcoreInputMsg->header.stamp;
	//transform to world frame
	geometry_msgs::PoseStamped phonePose;
	tf2::doTransform(phonePoseKinect,phonePose,transformKinectToWorld);
	phonePose.header.frame_id = "/world";

	//ROS_INFO_STREAM("estimated pose before transform: "<<phonePoseKinect.pose.position.x<<" "<<phonePoseKinect.pose.position.y<<" "<<phonePoseKinect.pose.position.z<<" ; "<<phonePoseKinect.pose.orientation.x<<" "<<phonePoseKinect.pose.orientation.y<<" "<<phonePoseKinect.pose.orientation.z<<" "<<phonePoseKinect.pose.orientation.w);

	pose_raw_pub.publish(phonePose);
	//publishPoseAsTfFrame(phonePose,"mobile_pose");


	ROS_INFO_STREAM("estimated pose is                "<<phonePose.pose.position.x<<" "<<phonePose.pose.position.y<<" "<<phonePose.pose.position.z<<" ; "<<phonePose.pose.orientation.x<<" "<<phonePose.pose.orientation.y<<" "<<phonePose.pose.orientation.z<<" "<<phonePose.pose.orientation.w);










	//:::::::::::::::Get arcore world frame of reference::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::











	tf::Pose phonePoseTf;
	tf::poseMsgToTF(phonePose.pose,phonePoseTf);
	ROS_INFO_STREAM("phonePoseTf = "<<poseToString(phonePoseTf));
	//Dim:
	// let phonePoseArcoreFrameConverted = Pa
	// let arcoreWorld = A
	// let phonePoseTf = Pr
	//
	// A*Pa*0 = Pr*0
	// so:
	// A*Pa = Pr
	// so:
	// A*Pa*Pa^-1 = Pr*Pa^-1
	// so:
	// A = Pr*Pa^-1
	tf::Pose arcoreWorld = phonePoseTf * phonePoseArcoreFrameConverted.inverse();
	publishTransformAsTfFrame(arcoreWorld,"arcore_world","/world",arcoreInputMsg->header.stamp);
	publishTransformAsTfFrame(phonePoseTf,"phone_estimate","/world",arcoreInputMsg->header.stamp);
	//publishTransformAsTfFrame(phonePoseArcoreFrameConverted,"phone_arcore_arcore","/arcore_world",arcoreInputMsg->header.stamp);
	//publishTransformAsTfFrame(phonePoseArcoreFrameConverted.inverse(),"phone_arcore_conv_inv","/phone_estimate",arcoreInputMsg->header.stamp);


	if(!isPoseValid(arcoreWorld))
	{
		ROS_WARN_STREAM("Dropping transform estimation as it is invalid");
		return;
	}

	tf::Pose phonePoseTfKinect;
	tf::poseMsgToTF(phonePoseKinect.pose,phonePoseTfKinect);
	double phoneToCameraRotationAngle = phonePoseTfKinect.getRotation().getAngleShortestPath()*180/3.14159;
	if(phoneToCameraRotationAngle>phoneOrientationDifferenceThreshold_deg)
	{
		ROS_INFO_STREAM("Orientation difference between phone and camera is too high, discarding estimation ("<<phoneToCameraRotationAngle<<")");
		return;
	}

	arcoreWorldHistory.push_back(arcoreWorld);


	//if this is one of the first few frames
	if(arcoreWorldHistory.size()<=startupFramesNum)
	{
		//for the first few frame just use the average
		tf::Vector3 averagePosition = averagePosePositions(arcoreWorldHistory);

		//if the next frame uses the kalman filter then initialize it with the estimate that is closest to the average

		tf::Pose closestEstimate;
		double minimumDistance = std::numeric_limits<double>::max();
		for(tf::Pose estimate : arcoreWorldHistory)
		{
			double distance = averagePosition.distance(estimate.getOrigin());
			if(distance<minimumDistance)
			{
				minimumDistance=distance;
				closestEstimate = estimate;
			}
		}

		if(arcoreWorldHistory.size()==startupFramesNum)
			transformKalmanFilter->update(closestEstimate);

		publishTransformAsTfFrame(closestEstimate,"arcore_world_filtered","/world",arcoreInputMsg->header.stamp);
		lastEstimate = closestEstimate;
	}
	else
	{
		if(poseDistance(arcoreWorld,lastEstimate)>estimateDistanceThreshold_meters)
		{
			ROS_INFO_STREAM("New estimation is too different, discarding frame");
			return;			
		}
		//if we are after the forst few frames use kalman
		tf::Pose filteredArcoreWorld = transformKalmanFilter->update(arcoreWorld);
		publishTransformAsTfFrame(filteredArcoreWorld,"arcore_world_filtered","/world",arcoreInputMsg->header.stamp);
		lastEstimate = filteredArcoreWorld;
		ROS_INFO_STREAM("filtering correction = "<<poseToString(arcoreWorld*filteredArcoreWorld.inverse()));
		ROS_INFO_STREAM("arcore_world_filtered = "<<poseToString(filteredArcoreWorld));
	}

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	unsigned long totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beginning).count();
	ROS_INFO("total duration is %lu ms",totalDuration);
}

















int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle nh;
	ROS_INFO_STREAM("starting "<<NODE_NAME);


	dynamic_reconfigure::Server<optar::OptarDynamicParametersConfig> server;
	dynamic_reconfigure::Server<optar::OptarDynamicParametersConfig>::CallbackType bindedDynamicParametersCallback;

	bindedDynamicParametersCallback = boost::bind(&dynamicParametersCallback, _1, _2);
	server.setCallback(bindedDynamicParametersCallback);


	transformKalmanFilter = std::make_shared<TransformKalmanFilter>(1e-5,1,1);


	boost::shared_ptr<sensor_msgs::CameraInfo const> kinectCameraInfoPtr;
	sensor_msgs::CameraInfo kinectCameraInfo;
	ROS_INFO_STREAM("waiting for kinect cameraInfo on topic "<<ros::names::remap(input_kinect_camera_info_topic));
	kinectCameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_kinect_camera_info_topic);
	if(kinectCameraInfoPtr == NULL)
	{
		ROS_ERROR("couldn't get kinect's camera_info, did the node shut down? I'll shut down.");
		return 0;
	}
	kinectCameraInfo = *kinectCameraInfoPtr;


	ros::Time targetTime;
	tf::TransformListener listener;
	std::string inputFrame = kinectCameraInfo.header.frame_id;
	std::string targetFrame = "/world";
	ros::Duration timeout = ros::Duration(10.0);
	bool retry=true;
	int count=0;
	ROS_INFO_STREAM("getting transform from "<<inputFrame<<" to "<<targetFrame);
	std::this_thread::sleep_for (std::chrono::seconds(2));//sleep two second to let tf start
	do
	{
		std::string failReason;
		targetTime = ros::Time(0);//the latest available
		bool r = listener.waitForTransform( targetFrame,inputFrame, targetTime, timeout, ros::Duration(0.01),&failReason);
		if(!r)
		{
			ROS_INFO_STREAM("can't transform because: "<<failReason);
			if(count>10)
				return -1;
			ROS_INFO("retrying");
		}
		else
			retry=false;
		count++;
	}while(retry);
	ROS_INFO("got transform");


	tf::StampedTransform transformKinectToWorldNotMsg = tf::StampedTransform();
	listener.lookupTransform(targetFrame, inputFrame, targetTime, transformKinectToWorldNotMsg);
	tf::transformStampedTFToMsg(transformKinectToWorldNotMsg,transformKinectToWorld);
	//ROS_INFO("got transform from %s to %s ",inputFrame.c_str(), targetFrame.c_str());


	pose_raw_pub = nh.advertise<geometry_msgs::PoseStamped>(output_pose_raw_topic, 10);
	pose_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(output_pose_marker_topic, 1);



	message_filters::Subscriber<opt_msgs::ArcoreCameraImage> arcoreCamera_sub(nh, input_arcore_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> kinect_img_sub(nh, input_kinect_camera_topic, 1);
	message_filters::Subscriber<sensor_msgs::Image> kinect_depth_sub(nh, input_kinect_depth_topic, 1);
	
	// Synchronization policy for having a callback that receives two topics at once.
	// It chooses the two messages by minimizing the time difference between them
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image, sensor_msgs::Image> MyApproximateSynchronizationPolicy;

	//instantiate and set up the policy
	MyApproximateSynchronizationPolicy policy = MyApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//We set a lower bound of half the period of the slower publisher, this should mek the algorithm behave better (according to the authors)
	policy.setInterMessageLowerBound(0,ros::Duration(0,250000000));// 2fps
	policy.setInterMessageLowerBound(1,ros::Duration(0,15000000));// about 30 fps but sometimes more
	policy.setInterMessageLowerBound(2,ros::Duration(0,15000000));// about 30 fps but sometimes more

	//Instantiate a Synchronizer with our policy.
	message_filters::Synchronizer<MyApproximateSynchronizationPolicy>  sync(MyApproximateSynchronizationPolicy(policy), arcoreCamera_sub, kinect_img_sub, kinect_depth_sub);

	//registers the callback
	auto f = boost::bind(&imagesCallback, _1, _2, _3, kinectCameraInfo);

	ROS_INFO_STREAM("waiting for images on topics: "<<endl<<
		ros::names::remap(input_arcore_topic)<<endl<<
		ros::names::remap(input_kinect_camera_topic)<<endl<<
		ros::names::remap(input_kinect_depth_topic)<<endl);
	sync.registerCallback(f);



	ros::spin();

	return 0;
}


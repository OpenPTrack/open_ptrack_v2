
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


using namespace cv;
using namespace std;


const string NODE_NAME 						= "nomarker_position_estimator";
const string input_arcore_topic				= "arcore_camera";
const string input_kinect_camera_topic		= "kinect_camera" ;
const string input_kinect_depth_topic		= "kinect_depth";
const string input_kinect_camera_info_topic	= "kinect_camera_info" ;


const string output_pose_raw_topic			= "optar/no_marker_pose_raw";
const string output_pose_marker_topic		= "optar/pose_marker";


tf::StampedTransform transformKinectToWorld;
ros::Publisher pose_raw_pub;
ros::Publisher pose_marker_pub;


//These may be overwritten by dynamic_reconfigure
double pnpReprojectionError = 5;
double pnpConfidence = 0.99;
double pnpIterations = 1000;
double matchingThreshold = 25;
double reprojectionErrorDiscardThreshold = 5;
int orbMaxPoints = 500;
double orbScaleFactor = 1.2;
int orbLevelsNumber = 8;


class PoseMatch
{
private:
	geometry_msgs::Pose mobilePose;
	geometry_msgs::Pose estimatedWorldPose;
	ros::Time time;
public:
	PoseMatch(const geometry_msgs::Pose& mobilePose_in, const geometry_msgs::Pose& estimatedWorldPose_in, const ros::Time& time_in):
		mobilePose(mobilePose_in),
		estimatedWorldPose(estimatedWorldPose_in),
		time(time_in)
	{

	}


	const geometry_msgs::Pose& getEstimatedPose()
	{
		return estimatedWorldPose;
	}

	const ros::Time& getTime()
	{
		return time;
	}
};

std::vector<PoseMatch> poseMatches;


/**
 * Extracts from the provided sequence of poses the longest sequence where
 * the movement from one pose to the next one doesn't require a speed higher than the provided one
 */
std::shared_ptr<std::vector<PoseMatch>> filterPosesBySpeed(std::vector<PoseMatch>& rawPoseMatches)
{
	/*
	ROS_INFO("-----------filterPosesBySpeed:-------------");
	for(unsigned int i=0;i<rawPoseMatches.size();i++)
	{
		ROS_INFO("    %f \t %f \t %f",rawPoseMatches.at(i).getEstimatedPose().position.x,rawPoseMatches.at(i).getEstimatedPose().position.y,rawPoseMatches.at(i).getEstimatedPose().position.z);
	}
	*/
	std::shared_ptr<std::vector<PoseMatch>> longestSequence = std::make_shared<std::vector<PoseMatch>>();

	std::vector<bool> isUsed(rawPoseMatches.size(),false);
	
	unsigned int startPoint = rawPoseMatches.size();
	do
	{
		startPoint--;
		std::shared_ptr<std::vector<PoseMatch>> candidate = std::make_shared<std::vector<PoseMatch>>();
		//keep track of which is the first unused pose, so on the next cycle we start from it
		//We also know that at the start of cycle all the poses before the start have been used
		//so the first unused is the first we skip
		//int firstUnused = startPoint+1;//in any case we will go at least one pose forward
		bool didSetFirstUnused=false;
		//ROS_INFO_STREAM("startPoint = "<<startPoint);
		for(unsigned int i=startPoint;;i--)
		{
			if(!isUsed.at(i))//if not already used
			{
				if(candidate->size()==0)//It's the first one, we accept it
				{
					isUsed[i]=true;
					candidate->push_back(rawPoseMatches[i]);
					//ROS_INFO("    %f \t %f \t %f accepted",candidate->back().getEstimatedPose().position.x,candidate->back().getEstimatedPose().position.y,candidate->back().getEstimatedPose().position.z);
				}
				else
				{
					//check the speed from the previous pose
					double distance = poseDistance(rawPoseMatches[i].getEstimatedPose(), candidate->back().getEstimatedPose());
					double timeDiff = (candidate->back().getTime() - rawPoseMatches[i].getTime()).toSec();

					if(distance/timeDiff<12*std::exp(-timeDiff)+3)
					{
						isUsed[i]=true;
						candidate->push_back(rawPoseMatches[i]);
						//ROS_INFO("    %f \t %f \t %f accepted",candidate->back().getEstimatedPose().position.x,candidate->back().getEstimatedPose().position.y,candidate->back().getEstimatedPose().position.z);
					}
				}
				/*if(!isUsed[i] && !didSetFirstUnused)//if we still didn't use this pose and we still havent set the first unused
				{
					firstUnused = i;
					didSetFirstUnused=true;
				}*/
			}
			else
			{
				//ROS_INFO("    %f \t %f \t %f already used",rawPoseMatches.at(i).getEstimatedPose().position.x,rawPoseMatches.at(i).getEstimatedPose().position.y,rawPoseMatches.at(i).getEstimatedPose().position.z);
			}
			if(i==0)
				break;
		}
		/*
		if(!didSetFirstUnused)
		{
			//this means there are no unused poses
			startPoint=rawPoseMatches.size();
		}
		else
		{
			startPoint=firstUnused;
		}
		*/
		/*
		ROS_INFO("candidate:");
		for(unsigned int i=0;i<candidate->size();i++)
		{
			ROS_INFO("    %f \t %f \t %f",candidate->at(i).getEstimatedPose().position.x,candidate->at(i).getEstimatedPose().position.y,candidate->at(i).getEstimatedPose().position.z);
		}*/
		if(candidate->size()>longestSequence->size())
		{
			longestSequence=candidate;
			//ROS_INFO("candidate is better");
		}
	}while(startPoint>0);

	//check everything went as planned, we should have used every pose
	for(unsigned int i=0;i<isUsed.size();i++)
	{
		if(!isUsed[i])
		{
			ROS_WARN("something went wrong, didn't check all the poses (%u is unused)",i);
		}
	}

	std::reverse(longestSequence->begin(),longestSequence->end());
	return longestSequence;
}


void dynamicParametersCallback(optar::OptarDynamicParametersConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f", 
            config.nomarker_position_estimator_pnp_iterations,
            config.nomarker_position_estimator_pnp_confidence, 
            config.nomarker_position_estimator_pnp_reprojection_error);
  pnpReprojectionError = config.nomarker_position_estimator_pnp_reprojection_error;
  pnpConfidence 					= config.nomarker_position_estimator_pnp_confidence;
  pnpIterations 					= config.nomarker_position_estimator_pnp_iterations;
  
  matchingThreshold 				= config.nomarker_position_estimator_matching_threshold;
  reprojectionErrorDiscardThreshold = config.nomarker_position_estimator_reprojection_discard_threshold;

  orbMaxPoints		= config.nomarker_position_estimator_orb_max_points;
  orbScaleFactor	= config.nomarker_position_estimator_orb_scale_factor;
  orbLevelsNumber	= config.nomarker_position_estimator_orb_levels_number;
}


int findOrbMatches(	const cv::Mat& arcoreImg, 
					const cv::Mat& kinectCameraImg, 
					std::vector<cv::DMatch>& matches, 
					std::vector<cv::KeyPoint>& arcoreKeypoints, 
					std::vector<cv::KeyPoint>& kinectKeypoints)
{
	matches.clear();
    cv::Ptr<cv::ORB> orb = cv::ORB::create(orbMaxPoints,orbScaleFactor,orbLevelsNumber/*500,1.189207115,12*/);

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
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(arcoreDescriptors,kinectDescriptors,matches);//arcore is query, kinect is trains
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


void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg,
					const sensor_msgs::ImageConstPtr& kinectInputCameraMsg,
					const sensor_msgs::ImageConstPtr& kinectInputDepthMsg,
					const sensor_msgs::CameraInfo& kinectCameraInfo)
{
	std::chrono::steady_clock::time_point beginning = std::chrono::steady_clock::now();
	long arcoreTime = arcoreInputMsg->header.stamp.sec*1000000000L + arcoreInputMsg->header.stamp.nsec;
	long kinectTime = kinectInputCameraMsg->header.stamp.sec*1000000000L + kinectInputCameraMsg->header.stamp.nsec;
	ROS_INFO("\n\n\n\n\n\nreceived images. time diff = %+7.5f sec.  arcore time = %012ld  kinect time = %012ld",(arcoreTime-kinectTime)/1000000000.0, arcoreTime, kinectTime);

	double arcoreCameraMatrixArr[kinectCameraInfo.P.size()] =	{	arcoreInputMsg->focal_length_x_px,	0, 									arcoreInputMsg->principal_point_x_px,
													0,									arcoreInputMsg->focal_length_y_px,	arcoreInputMsg->principal_point_y_px,
													0,									0,									1									};
	cv::Mat arcoreCameraMatrix = cv::Mat(3, 3, CV_64FC1, arcoreCameraMatrixArr);

	cv::Mat arcoreImg = cv::imdecode(cv::Mat(arcoreInputMsg->image.data),1);//convert compressed image data to cv::Mat
	if(!arcoreImg.data)
	{
		ROS_ERROR("couldn't decode arcore image");
		return;
	}
    //cv::imshow("Arcore raw", arcoreImg);
	if(arcoreImg.channels()!=3)
	{
		ROS_ERROR("Color image expected from arcore device, received something different");
		return;
	}
	cv::Mat planes[3];
	split(arcoreImg,planes);  // planes[2] is the red channel
	arcoreImg = planes[2];
	cv::Mat flippedArcoreImg;
	cv::flip(arcoreImg,flippedArcoreImg,0);
	arcoreImg=flippedArcoreImg;
	//cv::xphoto::createSimpleWB()->balanceWhite(flippedArcoreImg,arcoreImg);
	//cv::equalizeHist(flippedArcoreImg,arcoreImg);
    ROS_INFO("decoded arcore image");
    //cv::imshow("Arcore", arcoreImg);


	cv::Mat kinectCameraImg = cv_bridge::toCvShare(kinectInputCameraMsg)->image;//convert compressed image data to cv::Mat
	if(!kinectCameraImg.data)
	{
		ROS_ERROR("couldn't extract kinect camera opencv image");
		return;
	}
    ROS_INFO("decoded kinect camera image");
    //cv::imshow("Kinect", kinectCameraImg);


	cv::Mat kinectDepthImg = cv_bridge::toCvShare(kinectInputDepthMsg)->image;//convert compressed image data to cv::Mat
	if(!kinectDepthImg.data)
	{
		ROS_ERROR("couldn't extract kinect depth opencv image");
		return;
	}
    ROS_INFO("decoded kinect depth image");
  /*  cv::namedWindow("KinectDepth", cv::WINDOW_NORMAL);
	cv::resizeWindow("KinectDepth",1280,720);
    cv::imshow("KinectDepth", kinectDepthImg);
*/
	std::chrono::steady_clock::time_point afterDecoding = std::chrono::steady_clock::now();
	unsigned long decodingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDecoding - beginning).count();
	ROS_INFO("Images decoding and initialization took %lu ms",decodingDuration);






	//find matches
    std::vector<cv::DMatch> matches;
    std::vector<cv::KeyPoint>  arcoreKeypoints;
    std::vector<cv::KeyPoint>  kinectKeypoints;
    int r = findOrbMatches(arcoreImg, kinectCameraImg, matches, arcoreKeypoints, kinectKeypoints);
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
	std::vector< cv::DMatch > goodMatches;
	for( unsigned int i = 0; i < goodMatchesWithNull.size(); i++ )
	{
		//QueryIdx is for arcore descriptors, TrainIdx is for kinect. This is because of how we passed the arguments to BFMatcher::match
		cv::Point2f imgPos = kinectKeypoints.at(goodMatchesWithNull.at(i).trainIdx).pt;
		//try to find the depth using the closest pixel
		if(kinectDepthImg.at<uint16_t>(imgPos)==0)
		{
			Point2i nnz = findNearestNonZeroPixel(kinectDepthImg,imgPos.x,imgPos.y,100);
			ROS_INFO("Got closest non-zero pixel, %d;%d",nnz.x,nnz.y);
			kinectDepthImg.at<uint16_t>(imgPos)=kinectDepthImg.at<uint16_t>(nnz);
		}

		if(kinectDepthImg.at<uint16_t>(imgPos)==0)
		{
			ROS_INFO("Dropped match as it had zero depth");
		}
		else
		{
			goodMatches.push_back(goodMatchesWithNull.at(i));
		}
	}
  	ROS_INFO_STREAM("got "<<goodMatches.size()<<" actually good matches");








	visualization_msgs::MarkerArray markerArray;


    //find the 3d poses corrseponding to the goodMatches, these will be relative to the kinect frame
    std::vector<cv::Point3f> goodMatches3dPos;
    std::vector<cv::Point2f> goodMatchesImgPos;

	for( unsigned int i = 0; i < goodMatches.size(); i++ )
	{
		//QueryIdx is for arcore descriptors, TrainIdx is for kinect. This is because of how we passed the arguments to BFMatcher::match
		cv::Point2f kinectPixelPos = kinectKeypoints.at(goodMatches.at(i).trainIdx).pt;
		cv::Point2f arcorePixelPos = arcoreKeypoints.at(goodMatches.at(i).queryIdx).pt;
		ROS_DEBUG_STREAM("good match between "<<kinectPixelPos.x<<";"<<kinectPixelPos.y<<" \tand \t"<<arcorePixelPos.x<<";"<<arcorePixelPos.y<<" \tdistance = "<<goodMatches.at(i).distance);
		goodMatchesImgPos.push_back(arcorePixelPos);
		//ROS_INFO_STREAM("depth = "<<kinectDepthImg.at<uint16_t>(kinectPixelPos));
	/*	cv::rectangle(	kinectCameraImg,
						cv::Point((int)kinectPixelPos.x-10,(int)kinectPixelPos.y-10),
						cv::Point((int)kinectPixelPos.x+10,(int)kinectPixelPos.y+10),
						cv::Scalar(1,0,0),4);*/
		cv::Point3f pos3d = get3dPoint(	kinectPixelPos.x,kinectPixelPos.y,
												kinectDepthImg.at<uint16_t>(kinectPixelPos),
												kinectCameraInfo.P[0+4*0],kinectCameraInfo.P[1+4*1],kinectCameraInfo.P[2+4*0],kinectCameraInfo.P[2+4*1]);
		goodMatches3dPos.push_back(pos3d);

		visualization_msgs::Marker matchMarker;
		buildMarker(matchMarker,
					pos3d,
					"match"+std::to_string(i),
					0,0,1,1, 0.2, kinectInputCameraMsg->header.frame_id);//matches are blue
		markerArray.markers.push_back(matchMarker);
	}

	for(cv::Point2f pix : goodMatchesImgPos)
	{
		cv::rectangle(	arcoreImg,
						cv::Point((int)pix.x-10,(int)pix.y-10),
						cv::Point((int)pix.x+10,(int)pix.y+10),
						cv::Scalar(1,1,1),4);
	}
	



	if(goodMatches.size()<4)
	{
		cv::Mat matchesImg;
	    cv::drawMatches(arcoreImg, arcoreKeypoints, kinectCameraImg, kinectKeypoints, goodMatches, matchesImg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		prepareOpencvImageForShowing("Matches", matchesImg, 800);
	    cv::waitKey(10);

		pose_marker_pub.publish(markerArray);
		ROS_INFO("not enough matches to determine position");
		return;
	}


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

	std::vector<Point2f> reprojPoints;
	cv::projectPoints 	(goodMatches3dPos,
						rvec, tvec,
						arcoreCameraMatrix,
						cv::noArray(),
						reprojPoints);


	cv::Mat colorArcoreImg;
	cvtColor(arcoreImg, colorArcoreImg, CV_GRAY2RGB);
	double reprojError = 0;
	
	for(unsigned int i=0;i<inliers.size();i++)
	{
		Point2f pix = goodMatchesImgPos.at(inliers.at(i));
		Point2f reprojPix = reprojPoints.at(inliers.at(i));
		reprojError += hypot(pix.x-reprojPix.x, pix.y-reprojPix.y)/reprojPoints.size();
		cv::circle(colorArcoreImg,pix,15,Scalar(128,128,128),5);
		int r = ((double)rand())/RAND_MAX*255;
		int g= ((double)rand())/RAND_MAX*255;
		int b = ((double)rand())/RAND_MAX*255;
		Scalar color = Scalar(r,g,b);
		cv::line(colorArcoreImg,pix,reprojPix,color,3);
	}
	ROS_INFO_STREAM("inliers reprojection error = "<<reprojError);


	cv::Mat matchesImg;
    cv::drawMatches(arcoreImg, arcoreKeypoints, kinectCameraImg, kinectKeypoints, goodMatches, matchesImg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	
	prepareOpencvImageForShowing("Matches", matchesImg, 800);
	prepareOpencvImageForShowing("Reprojection", colorArcoreImg, 400);
	cv::waitKey(10);


	if(reprojError>reprojectionErrorDiscardThreshold)
	{
		ROS_WARN("Reprojection error beyond threshold, discarding frame");
		return;
	}


	Eigen::Vector3d position;
	Eigen::Quaterniond rotation;
	opencvPoseToEigenPose(rvec,tvec,position,rotation);

	geometry_msgs::Pose poseNotStamped;
	poseNotStamped.position.x = position.x();
	poseNotStamped.position.y = position.y();
	poseNotStamped.position.z = position.z();

	poseNotStamped.orientation.x = rotation.x();
	poseNotStamped.orientation.y = rotation.y();
	poseNotStamped.orientation.z = rotation.z();
	poseNotStamped.orientation.w = rotation.w();


	//invert the pose
	tf::Pose poseTf;
	tf::poseMsgToTF(poseNotStamped,poseTf);
	tf::poseTFToMsg(poseTf.inverse(),poseNotStamped);

	geometry_msgs::PoseStamped pose;
	pose.pose = poseNotStamped;

	pose.header.frame_id = "kinect01_rgb_optical_frame";
	pose.header.stamp = arcoreInputMsg->header.stamp;

	ROS_INFO_STREAM("estimated pose before transform: "<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<" ; "<<pose.pose.orientation.x<<" "<<pose.pose.orientation.y<<" "<<pose.pose.orientation.z<<" "<<pose.pose.orientation.w);

	
	geometry_msgs::TransformStamped transformMsg;
	tf::transformStampedTFToMsg(transformKinectToWorld,transformMsg);
	tf2::doTransform(pose,pose,transformMsg);
	pose.header.frame_id = "/world";

	poseMatches.push_back(PoseMatch(arcoreInputMsg->mobileFramePose, pose.pose, arcoreInputMsg->header.stamp));
	ROS_INFO_STREAM("estimated pose is                "<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<" ; "<<pose.pose.orientation.x<<" "<<pose.pose.orientation.y<<" "<<pose.pose.orientation.z<<" "<<pose.pose.orientation.w);


	pose_raw_pub.publish(pose);
	publishPoseAsTfFrame(pose,"mobile_pose");
	visualization_msgs::Marker markerRaw;
	buildMarker(markerRaw,	pose.pose,	"nomarker_raw_pose", 1,0,0,1, 0.2, pose.header.frame_id);//raw is red
	markerArray.markers.push_back(markerRaw);

/*
	std::shared_ptr<std::vector<PoseMatch>> filteredPoseMatches = filterPosesBySpeed(poseMatches);
	visualization_msgs::Marker markerFiltered;
	buildMarker(markerFiltered,
				filteredPoseMatches->back().getEstimatedPose(),
				"nomarker_filtered_pose",
				0,1,0,1, 0.25, "world");//filtered is green and bigger	
	ROS_INFO("filtered estimated pose is %f \t%f \t%f",filteredPoseMatches->back().getEstimatedPose().position.x,filteredPoseMatches->back().getEstimatedPose().position.y,filteredPoseMatches->back().getEstimatedPose().position.z);
	markerArray.markers.push_back(markerFiltered);
*/

	pose_marker_pub.publish(markerArray);


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
	transformKinectToWorld = tf::StampedTransform();
	listener.lookupTransform(targetFrame, inputFrame, targetTime, transformKinectToWorld);
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


#include <ros/ros.h>
#include <opt_msgs/ArcoreCameraImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xphoto.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <chrono>

#define ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME "nomarker_position_estimator"

ros::Publisher pub;

void imagesCallback(const opt_msgs::ArcoreCameraImageConstPtr& arcoreInputMsg, const sensor_msgs::ImageConstPtr& kinectInputMsg)
{
	std::chrono::steady_clock::time_point beginning = std::chrono::steady_clock::now();
	long arcoreTime = arcoreInputMsg->header.stamp.sec*1000000000L + arcoreInputMsg->header.stamp.nsec;
	long kinectTime = kinectInputMsg->header.stamp.sec*1000000000L + kinectInputMsg->header.stamp.nsec;
	ROS_INFO("received images. time diff = %+7.5f sec.  arcore time = %012ld  kinect time = %012ld",(arcoreTime-kinectTime)/1000000000.0, arcoreTime, kinectTime);

	cv::Mat arcoreImg = cv::imdecode(cv::Mat(arcoreInputMsg->image.data),1);//convert compressed image data to cv::Mat
	if(!arcoreImg.data)
	{
		ROS_ERROR("couldn't decode arcore image");
		return;
	}
	if(arcoreImg.channels()==3)
	{
		cv::cvtColor(arcoreImg,arcoreImg,cv::COLOR_BGR2GRAY);
	}
	cv::Mat flippedArcoreImg;
	cv::flip(arcoreImg,flippedArcoreImg,0);
	arcoreImg=flippedArcoreImg;
	//cv::xphoto::createSimpleWB()->balanceWhite(flippedArcoreImg,arcoreImg);
	cv::equalizeHist(flippedArcoreImg,arcoreImg);
    ROS_INFO("decoded arcore image");
    cv::imshow("Arcore", arcoreImg);
	cv::Mat kinectImg = cv_bridge::toCvShare(kinectInputMsg)->image;//convert compressed image data to cv::Mat
	if(!kinectImg.data)
	{
		ROS_ERROR("couldn't extract kinect opencv image");
		return;
	}
    ROS_INFO("decoded kinect image");
	std::chrono::steady_clock::time_point afterDecoding = std::chrono::steady_clock::now();
	unsigned long decodingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDecoding - beginning).count();
	ROS_INFO("Images decoding and initialization took %lu ms",decodingDuration);

    cv::imshow("Kinect", kinectImg);


    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::vector<cv::KeyPoint> kinectKeypoints;
    std::vector<cv::KeyPoint> arcoreKeypoints;
   	std::chrono::steady_clock::time_point beforeDetection = std::chrono::steady_clock::now();
    orb->detect(kinectImg, kinectKeypoints);
    orb->detect(arcoreImg, arcoreKeypoints);
   	std::chrono::steady_clock::time_point afterDetection = std::chrono::steady_clock::now();
    unsigned long detectionDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterDetection - beforeDetection).count();
	ROS_INFO("Keypoints detection took %lu ms",detectionDuration);
	ROS_INFO_STREAM("Found "<<arcoreKeypoints.size()<<" keypoints for arcore, "<<kinectKeypoints.size()<<" for kinect");
    cv::Mat kinectKeypointsImg;
    cv::Mat arcoreKeypointsImg;
    drawKeypoints(kinectImg, kinectKeypoints, kinectKeypointsImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    drawKeypoints(arcoreImg, arcoreKeypoints, arcoreKeypointsImg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    //cv::imshow("Kinect keypoints",kinectKeypointsImg);
    //cv::imshow("ARCore keypoints",arcoreKeypointsImg);


	if ( kinectKeypoints.empty() )
	{
		ROS_ERROR("No keypoints found for kinect");
    	cv::waitKey(10);
		return;
	}
	if ( arcoreKeypoints.empty() )
	{
		ROS_ERROR("No keypoints found for arcore");
    	cv::waitKey(10);
		return;
	}

   	std::chrono::steady_clock::time_point beforeDescriptors = std::chrono::steady_clock::now();
    cv::Mat kinectDescriptors;
    cv::Mat arcoreDescriptors;
    orb->compute(kinectImg,kinectKeypoints,kinectDescriptors);
	orb->compute(arcoreImg,arcoreKeypoints,arcoreDescriptors);
   	std::chrono::steady_clock::time_point aftereDescriptors = std::chrono::steady_clock::now();
	unsigned long descriptorsDuration = std::chrono::duration_cast<std::chrono::milliseconds>(aftereDescriptors - beforeDescriptors).count();
	ROS_INFO("Descriptors computation took %lu ms",descriptorsDuration);

	if ( kinectDescriptors.empty() )
	{
		ROS_ERROR("No descriptors for kinect");
    	cv::waitKey(10);
		return;
	}
	if ( arcoreDescriptors.empty() )
	{
		ROS_ERROR("no descriptors for arcore");
    	cv::waitKey(10);
		return;
	}
  	  
   	std::chrono::steady_clock::time_point beforeMatching = std::chrono::steady_clock::now();
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(arcoreDescriptors,kinectDescriptors,matches);
   	std::chrono::steady_clock::time_point afterMatching = std::chrono::steady_clock::now();
	unsigned long matchingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(afterMatching - beforeMatching).count();
	ROS_INFO("Descriptors matching took %lu ms",matchingDuration);


  	double max_dist = -10000000;
  	double min_dist = 10000000;

	//-- Quick calculation of max and min distances between keypoints
	for( unsigned int i = 0; i < matches.size(); i++ )
	{
		double dist = matches[i].distance;
		if( dist < min_dist )
			min_dist = dist;
		if( dist > max_dist )
			max_dist = dist;
	}
	ROS_INFO("Max match dist : %f \n", max_dist );
	ROS_INFO("Min match dist : %f \n", min_dist );
	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< cv::DMatch > good_matches;
	for( unsigned int i = 0; i < matches.size(); i++ )
	{
		if( matches[i].distance <= std::max(min_dist+(max_dist-min_dist)*0.2,10.0) )
		{
			good_matches.push_back( matches[i]);
		}
	}




    cv::Mat matchesImg;
    cv::drawMatches(arcoreImg, arcoreKeypoints, kinectImg, kinectKeypoints, good_matches, matchesImg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	unsigned long totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - beginning).count();
	ROS_INFO("total duration is %lu ms",totalDuration);


    cv::imshow("Matches",matchesImg);
    cv::waitKey(10);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);
	ros::NodeHandle nh;
	ROS_INFO_STREAM("starting "<<ARCORE_CAMERA_IMAGE_REPUBLISHER_NODE_NAME);

	message_filters::Subscriber<opt_msgs::ArcoreCameraImage> arcoreCamera_sub(nh, "arcore_camera", 1);
	message_filters::Subscriber<sensor_msgs::Image> kinect_sub(nh, "kinect_camera", 1);
	
	// Synchronization policy for having a callback that receives two topics at once.
	// It chooses the two messages by minimizing the time difference between them
	typedef message_filters::sync_policies::ApproximateTime<opt_msgs::ArcoreCameraImage, sensor_msgs::Image> MyApproximateSynchronizationPolicy;

	//instantiate and set up the policy
	MyApproximateSynchronizationPolicy policy = MyApproximateSynchronizationPolicy(60);//instatiate setting up the queue size
	//we know we will receive stuff from arcore once per second, so we tell the algorithm a lower bound of
	// half a second, as suggested by the authors. This should make it more effective
	policy.setInterMessageLowerBound(0,ros::Duration(0,500000000));

	//Instantiate a Synchronizer with our policy.
	message_filters::Synchronizer<MyApproximateSynchronizationPolicy>  sync(MyApproximateSynchronizationPolicy(policy), arcoreCamera_sub, kinect_sub);

	//registers the callback
	sync.registerCallback(boost::bind(&imagesCallback, _1, _2));



	ros::spin();

	return 0;
}


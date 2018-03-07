#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Publish Messages
#include <opt_msgs/RoiRect.h>
#include <opt_msgs/Rois.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <opt_msgs/Detection.h>
#include <opt_msgs/DetectionArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <yolo_detector/open_ptrack_yoloConfig.h>

extern  "C"{
#include "network.h"
#include "image.h"
#include "run_yolo_obj.h"
#include "parser.h"

#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "option_list.h"
}

#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"

#include <Eigen/Eigen>

#include <open_ptrack/opt_utils/conversions.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace opt_msgs;

typedef sensor_msgs::Image Image;

namespace enc = sensor_msgs::image_encodings;

network* net;
char **names;
image **alphabet;
box *boxes_y;
float **probs;

float thresh;
float hier_thresh;
float median_factor;

image_transport::Publisher pub;
ros::Publisher detection_pub;

open_ptrack::opt_utils::Conversions converter;

Eigen::Matrix3f intrinsics_matrix;
bool camera_info_available_flag = false;

double _cx;
double _cy;

double _constant_x;
double _constant_y; 

std::string encoding;
float mm_factor;

void camera_info_cb (const CameraInfo::ConstPtr & msg)
{
	intrinsics_matrix << msg->K[0], 0, msg->K[2], 0, msg->K[4], msg->K[5], 0, 0, 1;
	
	_cx = msg->K[2];
	_cy = msg->K[5];
	
	_constant_x =  1.0f / msg->K[0];
	_constant_y = 1.0f /  msg->K[4];
	
	camera_info_available_flag = true;
}

void dynamic_callback(yolo_detector::open_ptrack_yoloConfig &config, uint32_t level) 
{
	std::cout << "Adjusting Parameters" <<  std::endl;
	thresh = (float)config.ObjectThresh;
	hier_thresh = (float)config.ObjectHier_Thresh; 
	median_factor = (float)config.ObjectMedian_Factor;
		std::cout << "thresh:" << thresh << " hier_thresh:" << hier_thresh << " median_factor:" <<median_factor<<  std::endl; 
	
	
}

image ipl_to_image(IplImage* src)
{
    unsigned char *data = (unsigned char *)src->imageData;
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    int step = src->widthStep;
    image out = make_image(w, h, c);
    int i, j, k, count=0;;

    for(k= 0; k < c; ++k){
        for(i = 0; i < h; ++i){
            for(j = 0; j < w; ++j){
                out.data[count++] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return out;
}

image convert_image(cv_bridge::CvImageConstPtr cv_ptr_rgb, int w, int h)
{
	IplImage iplimg = cv_ptr_rgb->image;
	image out = ipl_to_image(&iplimg);
    rgbgr_image(out);
    
    if((h && w) && (h != out.h || w != out.w))
    {
        image resized = resize_image(out, w, h);
        free_image(out);
        out = resized;
    }
    
    return out;
}

float median(const cv::Mat& Input)
{

  std::vector<float> array;

  if (Input.isContinuous())
  {
    array.assign(Input.datastart, Input.dataend);

  } 
  else 
  {
    for (int i = 0; i < Input.rows; ++i) 
    {
      array.insert(array.end(), Input.ptr<float>(i), Input.ptr<float>(i)+Input. cols);
    }
  }

  std::nth_element(array.begin() , array.begin() + array.size() * 0.5, array.end());

  return array[array.size() * 0.5];
}

void callback(const Image::ConstPtr& rgb_image,
         const Image::ConstPtr& depth_image)
{
	//std::cout<<"Sync Started"<<std::endl;
    cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare(rgb_image,
                                                                   enc::BGR8);
    if((pub.getNumSubscribers() > 0 || detection_pub.getNumSubscribers()) && camera_info_available_flag)
    {
	//	std::cout<<"Run Yolo"<<std::endl;
		ros::Time begin = ros::Time::now();
		image im = convert_image(cv_ptr_rgb,0,0);
		
		//std::cout << "START CREATE BOX INFO" << std::endl;
		boxInfo* boxes = (boxInfo*)calloc(1, sizeof(boxInfo));
		boxes->num = 200;
		boxes->boxes = (adjBox*)calloc(200, sizeof(adjBox));
		
		//std::cout << "ENTER C CODE" << std::endl;
		run_yolo_detection_obj(im, net, boxes_y, probs, thresh,  hier_thresh, names, boxes);
		
		printf( "Yolo object count = %d\n", boxes->num);
		double duration = ros::Time::now().toSec() - begin.toSec();

		std::cout << "Yolo detection time: " << duration<< std::endl;
		
    	
    	//Get Depth Image
		cv::Mat _depth_image;
		cv_bridge::CvImage::Ptr cv_ptr_depth;
		try
		{
		    cv_ptr_depth = cv_bridge::toCvCopy(depth_image, encoding); //, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		catch (cv_bridge::Exception& e)
		{
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
		}

		_depth_image = cv_ptr_depth->image;
		
		DetectionArray::Ptr detection_array_msg(new DetectionArray);
		detection_array_msg->header = rgb_image->header;
		
		for(int i = 0; i < 3; i++)
		{
			for(int j = 0; j < 3; j++)
			{
				detection_array_msg->intrinsic_matrix.push_back(intrinsics_matrix(i, j));
			}
		}
		
		cv::Mat image = cv_ptr_rgb->image;
		
		detection_array_msg->confidence_type = std::string("yolo");
		detection_array_msg->image_type = std::string("rgb");
		
		int i;
		for(i = 0; i < boxes->num; i++)
		{
			int medianX = boxes->boxes[i].x + (boxes->boxes[i].w / 2);
			int medianY = boxes->boxes[i].y + (boxes->boxes[i].h / 2);
			
			int newX = medianX - (median_factor * (medianX - boxes->boxes[i].x));
			int newY = medianY - (median_factor * (medianY - boxes->boxes[i].y));
			int newWidth = 2 * (median_factor * (medianX - boxes->boxes[i].x));
			int newHeight = 2 * (median_factor * (medianY - boxes->boxes[i].y));
			
			
			cv::Rect rect(newX, newY, newWidth, newHeight);
			float medianDepth = median(_depth_image(rect)) / mm_factor;
			if (medianDepth > 6.25) {
				std::cout << "mediandepth " << medianDepth << " rejecting" << std::endl;
				continue;
			}			
//float medianDepth = _depth_image.at<float>(medianY, medianX) / 1000.0f;
			
			std::string object_name(names[boxes->boxes[i].classID]);  
				    
			std::stringstream ss;
			ss << object_name << ":" << medianDepth; //  << " " << mm_factor;
			
			
			if(pub.getNumSubscribers() > 0)
			{
				cv::rectangle(image, cv::Point( newX, newY ), cv::Point( newX+ newWidth, newY+ newHeight), cv::Scalar( 0, 255, 0 ), 4);
				cv::rectangle(image, cv::Point( boxes->boxes[i].x, boxes->boxes[i].y ), 
									 cv::Point( boxes->boxes[i].x+ boxes->boxes[i].w, boxes->boxes[i].y+ boxes->boxes[i].h), cv::Scalar( 255, 0, 255 ), 10);
				cv::putText(image, ss.str(), cv::Point(boxes->boxes[i].x+10,boxes->boxes[i].y+20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(200,200,250), 1, CV_AA);
			}
			
			float mx =  (medianX - _cx) * medianDepth * _constant_x;
			float my = (medianY - _cy) * medianDepth * _constant_y;
			
			if(std::isfinite(medianDepth) && std::isfinite(mx) && std::isfinite(my))
			{
				
				Detection detection_msg;
			
				detection_msg.box_3D.p1.x = mx;
				detection_msg.box_3D.p1.y = my;
				detection_msg.box_3D.p1.z = medianDepth;
			
				detection_msg.box_3D.p2.x = mx;
				detection_msg.box_3D.p2.y = my;
				detection_msg.box_3D.p2.z = medianDepth;
			
				detection_msg.box_2D.x = medianX;
				detection_msg.box_2D.y = medianY;
				detection_msg.box_2D.width = 0;
				detection_msg.box_2D.height = 0;
				detection_msg.height = 0;
				detection_msg.confidence = 10;
				detection_msg.distance = medianDepth;
			
				detection_msg.centroid.x = mx;
				detection_msg.centroid.y = my;
				detection_msg.centroid.z = medianDepth;
			
				detection_msg.top.x = 0;
				detection_msg.top.y = 0;
				detection_msg.top.z = 0;
			
				detection_msg.bottom.x = 0;
				detection_msg.bottom.y = 0;
				detection_msg.bottom.z = 0;
			
			    // jb 
			    // 
			    // Add for objects 
			    // 
			
 			    detection_msg.object_name=object_name; 
				//std::cout << object_name << std::endl; 
				// end add
				
				detection_array_msg->detections.push_back(detection_msg);
				
			}
		}
		
		if(pub.getNumSubscribers() > 0)
		{
			
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			pub.publish(msg);
    	}
		
		//std::cout << "publishing " << detection_array_msg << std::endl; 
		detection_pub.publish(detection_array_msg);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_based_object_detector");
    ros::NodeHandle nh("~");
	
	std::string depth_image_topic;
	nh.param("depth_image_topic", depth_image_topic, std::string("/camera/depth_registered/points"));
	std::string rgb_image_topic;
	nh.param("rgb_image_topic", rgb_image_topic, std::string("/camera/depth_registered/points"));
	std::string output_topic;
	nh.param("output_topic", output_topic, std::string("/objects_detector/detections"));  // jb per https://github.com/OpenPTrack/open_ptrack_v2/blob/master/detection/apps/multiple_objects_detection_node.cpp#L67
	std::string camera_info_topic;
	nh.param("camera_info_topic", camera_info_topic, std::string("/camera/rgb/camera_info"));
	
	std::string encoding_param;
	nh.param("encoding_type", encoding_param, std::string("16UC1"));
	
	int in_mm;
	nh.param("in_mm", in_mm, 0);
	
	if(encoding_param.compare(std::string("16UC1")) == 0)
	{
		encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	}
	else if(encoding_param.compare(std::string("32FC1")) == 0)
	{
		encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	}
	else
	{
		encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	}
	
	if(in_mm)
	{
		mm_factor = 1000.0f;
	}
	else
	{
		mm_factor = 1.0f;
	}
	
	double thresh_;
	double hier_thresh_;
	double median_factor_; 
	median_factor = 0.1;
	camera_info_available_flag = false;
	
	// These have defaults set in too many places. 
	// Here, then in the config file, then in the dynamic config file.  The dynamic configuration is always called, so set there and then rebuild :(
	nh.param("thresh", thresh_, 0.25);
	nh.param("hier_thresh", hier_thresh_, 0.5);
		nh.param("median_factor", median_factor_, 0.1);
	
	
	thresh = (float)thresh_;
	hier_thresh = (float)hier_thresh_;
	median_factor = (float)median_factor_;
	
std::string datacfg;
	nh.param("data_cfg", datacfg, std::string("cfg/coco.data"));// overridden
	std::string cfgfile;
	nh.param("yolo_cfg", cfgfile, std::string("cfg/yolo.cfg"));  // overridden
	std::string weightfile;
	nh.param("weight_file", weightfile, std::string("yolo.weights"));// overridden
		std::string namefile;
	nh.param("name_file", namefile, std::string("data/coco.names"));// overridden
	
	
	std::string root_str;
	nh.param("root", root_str, std::string("home"));
	
	// revise to new API 
    net = parse_network_cfg( (char*)cfgfile.c_str() );
	char *arr = (char*)((void*) &(net->layers[0]));

	printf("exit");
    //printf( "detect layer  w = %d h = %d n = %d max = %d\n",  ((layer)arr[(net.n - 1)*sizeof_layer()]).w, net.layers[net.n-1].h, net.layers[net.n-1].n, net.layers[net.n-1].w*net.layers[net.n-1].h*net.layers[net.n-1].n );
	//printf( "detect layer  w = %d h = %d n = %d max = %d\n",  layers.w, layers.h, layers.n, layers.w*layers.h*layers.n );
    load_weights( net, (char*)weightfile.c_str() );
    
    
    set_batch_network( net, 1 );
	srand(2222222);
	
	boxes_y = init_boxes_obj(net);
	probs = init_probs_obj(net);
	
	// jb - there was a problem with parsing this, and we may not need rest of config file, so put here - 
	// list *options = read_data_cfg((char*)datacfg.c_str() );
//     char *name_list = option_find_str(options, "names",  "data/coco.names"); // "data/open_ptrack_object.names");// not overridden to do
//     
    std::string name_list_str(namefile);
//    name_list_str = root_str + "/" + name_list_str;
    
    std::string data_list_str = root_str + "/data";
    std::cout<<data_list_str<<std::endl;
    
    
    names = get_labels((char*)name_list_str.c_str());
    image **alphabet = load_alphabet_obj_((char*)data_list_str.c_str());

	std::cout<<"YOLO Set UP - objects"<<std::endl;
		std::cout << "thresh:" << thresh << " hier_thresh" << hier_thresh << std::endl; 
	
	
	image_transport::ImageTransport it(nh);
    pub = it.advertise("yolo_object_detector/image", 1);
    detection_pub= nh.advertise<DetectionArray>(output_topic, 3);
    
    dynamic_reconfigure::Server<yolo_detector::open_ptrack_yoloConfig> server;
    dynamic_reconfigure::Server<yolo_detector::open_ptrack_yoloConfig>::CallbackType f;
    
    f = boost::bind(&dynamic_callback, _1, _2);
    server.setCallback(f);
	
    message_filters::Subscriber<Image> rgb_image_sub(nh, rgb_image_topic, 1);
    message_filters::Subscriber<Image> depth_image_sub(nh, depth_image_topic, 1);
    
    ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, camera_info_cb);

    typedef sync_policies::ApproximateTime<Image, Image> Sync1Policy;
    
    Synchronizer<Sync1Policy> sync(Sync1Policy(10), rgb_image_sub, depth_image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    free(net);
    return 0;
}

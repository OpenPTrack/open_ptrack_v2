#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <utility> //std::pair
#include <condition_variable>
#include <thread>

#include <pthread.h>
#include <time.h>
#include <signal.h>
#include <stdio.h>  // snprintf
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>

#include <boost/thread/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/MarkerArray.h>

#include <rtpose_wrapper/Joint2DMsg.h>
#include <rtpose_wrapper/Joint3DMsg.h>
#include <rtpose_wrapper/SkeletonMsg.h>
#include <rtpose_wrapper/SkeletonArrayMsg.h>

#include <dynamic_reconfigure/server.h>
#include <rtpose_wrapper/JointsConfig.h>

#include <eigen3/Eigen/Eigen>

#include "caffe/cpm/frame.h"
#include "caffe/cpm/layers/imresize_layer.hpp"
#include "caffe/cpm/layers/nms_layer.hpp"
#include "caffe/net.hpp"
#include "caffe/util/blocking_queue.hpp"
#include "rtpose/modelDescriptor.h"
#include "rtpose/modelDescriptorFactory.h"
#include "rtpose/renderFunctions.h"


std::string _camera_info_topic;
std::string _rgb_topic;
std::string _depth_topic;
std::mutex _callback_mutex;
double _fx, _fy, _cx, _cy, _constant_x, _constant_y;
bool _not_ready = true;
sensor_msgs::Image _rgb_frame_from_sensor;
sensor_msgs::Image _depth_frame_from_sensor;
std_msgs::Header _rgb_header;
cv::Mat _depth_image, _depth_image_copy, _colored_depth_image;
sensor_msgs::CameraInfo _camera_info;
bool _head_joint_enhancement = true;
double _head_correction_param;

int _median_search_y_component;
int _median_search_x_component;

int _max_size_input_queue;

bool _fullscreen;
int _part_to_show;
std::string _write_frames;
bool _no_frame_drops;
std::string _write_json;
int _camera_index;
std::string _video;
std::string _image_dir;
int _start_frame;
std::string _caffe_model;
std::string _caffe_proto;
std::string _resolution;
std::string _net_resolution;
std::string _camera_resolution;
int _start_device;
int _num_gpu;
double _start_scale;
double _scale_gap;
int _num_scales;
bool _no_display;
bool _skeleton_solid_color;
cv::Point3d _skeleton_color;
std::string _skeleton_topic_to_publish;
std::string _marker_topic_to_publish;
ros::Publisher _skeleton_pub;
ros::Publisher _raw_skeleton_image_pub, _very_raw_image_pub;
ros::Publisher _raw_skeleton_depth_image_pub;
double _noise_threshold;
int _minimum_number_of_valid_3D_joints;
std::string _raw_skeleton_image_topic_to_publish;
bool _raw_skeleton_image_on_topic;
double _min_confidence_per_joint;

int debug_count =0;

// Global parameters
int DISPLAY_RESOLUTION_WIDTH;
int DISPLAY_RESOLUTION_HEIGHT;
int CAMERA_FRAME_WIDTH;
int CAMERA_FRAME_HEIGHT;
int NET_RESOLUTION_WIDTH;
int NET_RESOLUTION_HEIGHT;
int BATCH_SIZE;
double SCALE_GAP;
double START_SCALE;
int NUM_GPU;
std::string PERSON_DETECTOR_CAFFEMODEL; //person detector
std::string PERSON_DETECTOR_PROTO;      //person detector
std::string POSE_ESTIMATOR_PROTO;       //pose estimator
const auto MAX_PEOPLE = RENDER_MAX_PEOPLE;  // defined in render_functions.hpp
const auto BOX_SIZE = 368;
const auto BUFFER_SIZE = 1;    //affects latency
const auto MAX_NUM_PARTS = 70;
#define DEBUG 0

// global queues for I/O
struct Global {
  caffe::BlockingQueue<Frame> input_queue; //have to pop
  std::condition_variable input_queue_cond;
  std::mutex input_queue_mutex;
  caffe::BlockingQueue<Frame> output_queue; //have to pop
  std::condition_variable output_queue_cond;
  std::mutex output_queue_mutex;
  caffe::BlockingQueue<Frame> output_queue_ordered;
  std::condition_variable output_queue_ordered_cond;
  std::mutex output_queue_ordered_mutex;
  caffe::BlockingQueue<Frame> output_queue_mated;
  std::condition_variable output_queue_mated_cond;
  std::mutex output_queue_mated_mutex;
  std::condition_variable get_frames_cond;
  std::mutex get_frames_mutex;
  std::priority_queue<int, std::vector<int>, std::greater<int> > dropped_index;
  std::vector< std::string > image_list;
  std::mutex mutex;
  int part_to_show;
  bool quit_threads;
  // Parameters
  float nms_threshold;
  int connect_min_subset_cnt;
  float connect_min_subset_score;
  float connect_inter_threshold;
  int connect_inter_min_above_threshold;

  struct UIState {
    UIState() :
      is_fullscreen(0),
      is_video_paused(0),
      is_shift_down(0),
      is_googly_eyes(0),
      current_frame(0),
      seek_to_frame(-1),
      fps(0) {}
    bool is_fullscreen;
    bool is_video_paused;
    bool is_shift_down;
    bool is_googly_eyes;
    int current_frame;
    int seek_to_frame;
    double fps;
  };
  UIState uistate;
};

// network copy for each gpu thread
struct NetCopy {
  caffe::Net<float> *person_net;
  std::vector<int> num_people;
  int nblob_person;
  int nms_max_peaks;
  int nms_num_parts;
  std::unique_ptr<ModelDescriptor> up_model_descriptor;
  float* canvas; // GPU memory
  float* joints; // GPU memory
};

enum SkeletonJointId
{
  HEAD = 0,
  NECK,
  RSHOULDER,
  RELBOW,
  RWRIST,
  LSHOULDER,
  LELBOW,
  LWRIST,
  RHIP,
  RKNEE,
  RANKLE,
  LHIP,
  LKNEE,
  LANKLE,
  CHEST
} ;

// Used for mantaining information about the depth when the RGB is retrieved
std::map<int, cv::Mat> _depth_queue;
std::map<int, std_msgs::Header> _header_queue;
std::recursive_mutex _depth_queue_mutex;

struct ColumnCompare
{
  bool operator()(const std::vector<double>& lhs,
                  const std::vector<double>& rhs) const
  {
    return lhs[2] > rhs[2];
    //return lhs[0] > rhs[0];
  }
};

Global global;
std::vector<NetCopy> net_copies;

int rtcpm();
bool handleKey(int c);
void warmup(int);
void process_and_pad_image(float* target, cv::Mat oriImg, int tw, int th,
                           bool normalize);

geometry_msgs::Point
getGeomMsgsPoint(const rtpose_wrapper::Joint3DMsg& j)
{
  geometry_msgs::Point p;
  p.x = j.x; p.y = j.y; p.z = j.z;
  return p;
}

//void
//updateSkeletonMarkerArray(
//    visualization_msgs::MarkerArray& skeleton_markers_array,
//    const rtpose_wrapper::SkeletonMsg& skeleton,
//    size_t skel_id
//    )
//{
//  const std::array<std::pair<SkeletonJointId, SkeletonJointId>, 14>
//  LINKS =
//  {
//    std::pair<SkeletonJointId, SkeletonJointId> (CHEST, NECK),
//    std::pair<SkeletonJointId, SkeletonJointId> (NECK, RSHOULDER),
//    std::pair<SkeletonJointId, SkeletonJointId> (RSHOULDER, RELBOW),
//    std::pair<SkeletonJointId, SkeletonJointId> (RELBOW, RWRIST),
//    std::pair<SkeletonJointId, SkeletonJointId> (NECK, LSHOULDER),
//    std::pair<SkeletonJointId, SkeletonJointId> (LSHOULDER, LELBOW),
//    std::pair<SkeletonJointId, SkeletonJointId> (LELBOW, LWRIST),
//    std::pair<SkeletonJointId, SkeletonJointId> (CHEST, RHIP),
//    std::pair<SkeletonJointId, SkeletonJointId> (RHIP, RKNEE),
//    std::pair<SkeletonJointId, SkeletonJointId> (RKNEE, RANKLE),
//    std::pair<SkeletonJointId, SkeletonJointId> (CHEST, LHIP),
//    std::pair<SkeletonJointId, SkeletonJointId> (LHIP, LKNEE),
//    std::pair<SkeletonJointId, SkeletonJointId> (LKNEE, LANKLE),
//    std::pair<SkeletonJointId, SkeletonJointId> (NECK, HEAD)
//  };

//  // Joint Markers
//  for(int i = 0, end = skeleton.joints.size(); i != end; ++i)
//  {
//    const rtpose_wrapper::Joint3DMsg& j = skeleton.joints[i];
//    if ( not std::isfinite(j.x) or not std::isfinite(j.y)
//         or not std::isfinite(j.z)) continue;
//    visualization_msgs::Marker joint_marker;
//    joint_marker.header = skeleton.header;
//    joint_marker.ns = "joints";
//    joint_marker.id = i + skel_id * skeleton.joints.size() * 2;
//    joint_marker.type = visualization_msgs::Marker::CUBE;
//    joint_marker.action = visualization_msgs::Marker::ADD;
//    joint_marker.pose.position.x = j.x;
//    joint_marker.pose.position.y = j.y;
//    joint_marker.pose.position.z = j.z;
//    joint_marker.pose.orientation.x = 0.0;
//    joint_marker.pose.orientation.y = 0.0;
//    joint_marker.pose.orientation.z = 0.0;
//    joint_marker.pose.orientation.w = 1.0;
//    joint_marker.scale.x = 0.06;
//    joint_marker.scale.y = 0.06;
//    joint_marker.scale.z = 0.06;
//    //      joint_marker.color.r = color_(2);
//    //      joint_marker.color.g = color_(1);
//    //      joint_marker.color.b = color_(0);
//    joint_marker.color.r = 1.0;
//    joint_marker.color.g = 0.0;
//    joint_marker.color.b = 0.0;
//    joint_marker.color.a = 1.0;

//    joint_marker.lifetime = ros::Duration(0.2);

//    skeleton_markers_array.markers.push_back(joint_marker);
//  }
//  // Link markers
//  for(auto it = LINKS.begin(), end = LINKS.end();
//      it != end; ++it)
//  {
//    const rtpose_wrapper::Joint3DMsg& p1 = skeleton.joints[it->first];
//    const rtpose_wrapper::Joint3DMsg& p2 = skeleton.joints[it->second];
//    if( std::isfinite(p1.x + p1.y + p1.z)
//        and
//        std::isfinite(p2.x + p2.y + p2.z))
//    {
//      visualization_msgs::Marker line_marker;
//      line_marker.header = skeleton.header;
//      line_marker.ns = "links";
//      line_marker.id = (it - LINKS.begin()) +
//          skel_id * skeleton.joints.size() * 2
//          + skeleton.joints.size();
//      line_marker.type = visualization_msgs::Marker::LINE_STRIP;
//      line_marker.action = visualization_msgs::Marker::ADD;
//      line_marker.pose.orientation.x = 0.0;
//      line_marker.pose.orientation.y = 0.0;
//      line_marker.pose.orientation.z = 0.0;
//      line_marker.pose.orientation.w = 1.0;
//      line_marker.scale.x = 0.1;
//      line_marker.scale.y = 0.1;
//      line_marker.scale.z = 0.1;
//      line_marker.color.r = 1.0;
//      line_marker.color.g = 0.0;
//      line_marker.color.b = 0.0;
//      line_marker.color.a = 1.0;

//      line_marker.lifetime = ros::Duration(0.2);
//      line_marker.points.push_back(getGeomMsgsPoint(p1));
//      line_marker.points.push_back(getGeomMsgsPoint(p2));
//      std_msgs::ColorRGBA red;
//      red.a = red.r = 1.0;
//      line_marker.colors.push_back(red);
//      line_marker.colors.push_back(red);
//      skeleton_markers_array.markers.push_back(line_marker);
//    }
//  }
//}


double get_wall_time() {
  struct timeval time;
  if (gettimeofday(&time,NULL)) {
    //  Handle error
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * 1e-6;
  //return (double)time.tv_usec;
}

double medianBetweenValidPoints(const cv::Mat& Input){
  std::vector<ushort> array;
  if (Input.isContinuous()) {
    array.assign(Input.datastart, Input.dataend);
  } else {
    for (int i = 0; i < Input.rows; ++i) {
      array.insert(array.end(), Input.ptr<ushort>(i),
                   Input.ptr<ushort>(i)+Input.cols);
    }
  }
  // jump over the 0s
  std::vector<ushort> array_with_no_zero;
  array_with_no_zero.reserve(array.size());
  for(uint i = 0; i < array.size(); ++i)
  {
    if (not fabs(array[i]) < 0.01f)
    {
      array_with_no_zero.push_back(array[i]);
    }
  }
  if (array_with_no_zero.size() == 0) return 0.0f;
  auto nth_position = array_with_no_zero.begin() +
      array_with_no_zero.size() / 2;
  std::nth_element(array_with_no_zero.begin(),
                   nth_position,
                   array_with_no_zero.end());
  return array_with_no_zero[array_with_no_zero.size() * 0.5];
}
double medianBetweenValidPoints(std::vector<double>& array){
  std::vector<double> array2;
  array2.reserve(array.size());
  for(uint k = 0; k < array.size(); ++k)
  {
    if (not fabs(array[k]) < 0.01f)
    {
      array2.push_back(array[k]);
    }
  }
  std::nth_element(array2.begin(), array2.begin() + array2.size() * 0.5,
                   array2.end());
  return array2[array2.size() * 0.5];
}

bool allZeros(const double* vect)
{
  return fabs(vect[0] - 0.0f) < 0.1f
      and fabs(vect[1] - 0.0f) < 0.1f;
}

void
callback(rtpose_wrapper::JointsConfig &config, uint32_t level)
{
  _min_confidence_per_joint = config.min_confidence;
}

void syncronizedCallback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg,
                         const sensor_msgs::ImageConstPtr& rgb_msg,
                         const sensor_msgs::ImageConstPtr& depth_msg)
{
  ROS_INFO_STREAM_COND(DEBUG > 1, "CBK: wait");
  std::lock_guard<std::mutex> lock(_callback_mutex);
  ROS_INFO_STREAM_COND(DEBUG > 1, "CBK: locking");
  _depth_frame_from_sensor = *depth_msg;
  _rgb_frame_from_sensor = *rgb_msg;
  _not_ready = false;
  _camera_info = *camera_info_msg;
  DISPLAY_RESOLUTION_HEIGHT = CAMERA_FRAME_HEIGHT =
      _rgb_frame_from_sensor.height;
  DISPLAY_RESOLUTION_WIDTH = CAMERA_FRAME_WIDTH = _rgb_frame_from_sensor.width;
  ROS_INFO_STREAM_COND(DEBUG > 1,"CBK: unlocking");
}

void warmup(int device_id) {
  int logtostderr = FLAGS_logtostderr;

  LOG(INFO) << "Setting GPU " << device_id;

  caffe::Caffe::SetDevice(device_id); //cudaSetDevice(device_id) inside
  caffe::Caffe::set_mode(caffe::Caffe::GPU); //

  LOG(INFO) << "GPU " << device_id << ": copying to person net";
  FLAGS_logtostderr = 0;
  net_copies[device_id].person_net = new caffe::Net<float>
      (PERSON_DETECTOR_PROTO, caffe::TEST);
  net_copies[device_id].person_net->CopyTrainedLayersFrom
      (PERSON_DETECTOR_CAFFEMODEL);

  net_copies[device_id].nblob_person = net_copies[device_id].person_net->
      blob_names().size();
  net_copies[device_id].num_people.resize(BATCH_SIZE);
  const std::vector<int> shape { {BATCH_SIZE, 3, NET_RESOLUTION_HEIGHT,
          NET_RESOLUTION_WIDTH} };

  net_copies[device_id].person_net->blobs()[0]->Reshape(shape);
  net_copies[device_id].person_net->Reshape();
  FLAGS_logtostderr = logtostderr;

  caffe::NmsLayer<float> *nms_layer =
      (caffe::NmsLayer<float>*)net_copies[device_id].person_net->
      layer_by_name("nms").get();
  net_copies[device_id].nms_max_peaks = nms_layer->GetMaxPeaks();


  caffe::ImResizeLayer<float> *resize_layer =
      (caffe::ImResizeLayer<float>*)net_copies[device_id].person_net->
      layer_by_name("resize").get();

  resize_layer->SetStartScale(START_SCALE);
  resize_layer->SetScaleGap(SCALE_GAP);
  LOG(INFO) << "start_scale = " << START_SCALE;

  net_copies[device_id].nms_max_peaks = nms_layer->GetMaxPeaks();

  net_copies[device_id].nms_num_parts = nms_layer->GetNumParts();
  CHECK_LE(net_copies[device_id].nms_num_parts, MAX_NUM_PARTS)
      << "num_parts in NMS layer (" << net_copies[device_id].nms_num_parts
      << ") "
      << "too big ( MAX_NUM_PARTS )";

  if (net_copies[device_id].nms_num_parts==15) {
    ModelDescriptorFactory::createModelDescriptor
        (ModelDescriptorFactory::Type::MPI_15,
         net_copies[device_id].up_model_descriptor);
    global.nms_threshold = 0.2;
    global.connect_min_subset_cnt = 3;
    global.connect_min_subset_score = 0.4;
    global.connect_inter_threshold = 0.01;
    global.connect_inter_min_above_threshold = 8;
    LOG(INFO) << "Selecting MPI model.";
  } else if (net_copies[device_id].nms_num_parts==18) {
    ModelDescriptorFactory::createModelDescriptor
        (ModelDescriptorFactory::Type::COCO_18,
         net_copies[device_id].up_model_descriptor);
    global.nms_threshold = 0.05;
    global.connect_min_subset_cnt = 3;
    global.connect_min_subset_score = 0.4;
    global.connect_inter_threshold = 0.050;
    global.connect_inter_min_above_threshold = 9;
  } else {
    CHECK(0) << "Unknown number of parts! Couldn't set model";
  }

  //dry run
  LOG(INFO) << "Dry running...";
  net_copies[device_id].person_net->ForwardFrom(0);
  LOG(INFO) << "Success.";
  cudaMalloc(&net_copies[device_id].canvas, DISPLAY_RESOLUTION_WIDTH
             * DISPLAY_RESOLUTION_HEIGHT * 3 * sizeof(float));
  cudaMalloc(&net_copies[device_id].joints, MAX_NUM_PARTS*3*MAX_PEOPLE
             * sizeof(float) );
}

void process_and_pad_image(float* target, cv::Mat oriImg, int tw,
                           int th, bool normalize) {
  int ow = oriImg.cols;
  int oh = oriImg.rows;
  int offset2_target = tw * th;

  int padw = (tw-ow)/2;
  int padh = (th-oh)/2;
  //LOG(ERROR) << " padw " << padw << " padh " << padh;
  CHECK_GE(padw,0) << "Image too big for target size.";
  CHECK_GE(padh,0) << "Image too big for target size.";
  //parallel here
  unsigned char* pointer = (unsigned char*)(oriImg.data);

  for(int c = 0; c < 3; c++) {
    for(int y = 0; y < th; y++) {
      int oy = y - padh;
      for(int x = 0; x < tw; x++) {
        int ox = x - padw;
        if (ox>=0 && ox < ow && oy>=0 && oy < oh ) {
          if (normalize)
            target[c * offset2_target + y * tw + x] =
                float(pointer[(oy * ow + ox) * 3 + c])/256.0f - 0.5f;
          else
            target[c * offset2_target + y * tw + x] =
                float(pointer[(oy * ow + ox) * 3 + c]);
        }
        else {
          target[c * offset2_target + y * tw + x] = 0;
        }
      }
    }
  }
}

void render(int gid, float *heatmaps /*GPU*/) {
  float* centers = 0;
  float* poses    = net_copies[gid].joints;

  double tic = get_wall_time();
  if (net_copies[gid].up_model_descriptor->get_number_parts()==15) {
    render_mpi_parts(net_copies[gid].canvas, DISPLAY_RESOLUTION_WIDTH,
                     DISPLAY_RESOLUTION_HEIGHT, NET_RESOLUTION_WIDTH,
                     NET_RESOLUTION_HEIGHT,
                     heatmaps, BOX_SIZE, centers, poses,
                     net_copies[gid].num_people, global.part_to_show);
  } else if (net_copies[gid].up_model_descriptor->get_number_parts()==18) {
    if (global.part_to_show-1<=net_copies[gid].up_model_descriptor->
        get_number_parts()) {
      render_coco_parts(net_copies[gid].canvas,
                        DISPLAY_RESOLUTION_WIDTH, DISPLAY_RESOLUTION_HEIGHT,
                        NET_RESOLUTION_WIDTH, NET_RESOLUTION_HEIGHT,
                        heatmaps, BOX_SIZE, centers, poses,
                        net_copies[gid].num_people, global.part_to_show,
                        global.uistate.is_googly_eyes);
    } else {
      int aff_part = ((global.part_to_show-1)-net_copies[gid].
                      up_model_descriptor->get_number_parts()-1)*2;
      int num_parts_accum = 1;
      if (aff_part==0) {
        num_parts_accum = 19;
      } else {
        aff_part = aff_part-2;
      }
      aff_part += 1+net_copies[gid].up_model_descriptor->get_number_parts();
      render_coco_aff(net_copies[gid].canvas, DISPLAY_RESOLUTION_WIDTH,
                      DISPLAY_RESOLUTION_HEIGHT, NET_RESOLUTION_WIDTH,
                      NET_RESOLUTION_HEIGHT,
                      heatmaps, BOX_SIZE, centers, poses,
                      net_copies[gid].num_people, aff_part, num_parts_accum);
    }
  }
  VLOG(2) << "Render time " << (get_wall_time()-tic)*1000.0 << " ms.";
}

void* getFrameFromDir(void *i) {
  int global_counter = 1;
  int frame_counter = 0;
  cv::Mat image_uchar;
  cv::Mat image_uchar_orig;
  cv::Mat image_uchar_prev;
  while(1) {
    // If the queue is too long, wait for a bit
    std::unique_lock<std::mutex> lk(global.get_frames_mutex);
    global.get_frames_cond.wait(lk, [](){return
          global.input_queue.size() < _max_size_input_queue;});
    if (global.quit_threads or !ros::ok()) break;
    // Keep a count of how many frames we've seen in the video
    frame_counter++;

    // This should probably be protected.
    global.uistate.current_frame = frame_counter-1;

    std::string filename = global.image_list[global.uistate.current_frame];
    image_uchar_orig = cv::imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
    double scale = 0;
    if (image_uchar_orig.cols/(double)image_uchar_orig.rows>
        DISPLAY_RESOLUTION_WIDTH/(double)DISPLAY_RESOLUTION_HEIGHT) {
      scale = DISPLAY_RESOLUTION_WIDTH/(double)image_uchar_orig.cols;
    } else {
      scale = DISPLAY_RESOLUTION_HEIGHT/(double)image_uchar_orig.rows;
    }
    cv::Mat M = cv::Mat::eye(2,3,CV_64F);
    M.at<double>(0,0) = scale;
    M.at<double>(1,1) = scale;
    cv::warpAffine(image_uchar_orig, image_uchar, M,
                   cv::Size(DISPLAY_RESOLUTION_WIDTH,
                            DISPLAY_RESOLUTION_HEIGHT),
                   CV_INTER_CUBIC,
                   cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    // resize(image_uchar, image_uchar, cv::Size(new_width, new_height), 0, 0,
    // CV_INTER_CUBIC);
    image_uchar_prev = image_uchar;

    if ( image_uchar.empty() ) continue;

    Frame frame;
    frame.ori_width = image_uchar_orig.cols;
    frame.ori_height = image_uchar_orig.rows;
    frame.index = global_counter++;
    frame.video_frame_number = global.uistate.current_frame;
    frame.data_for_wrap = new unsigned char [DISPLAY_RESOLUTION_HEIGHT
        * DISPLAY_RESOLUTION_WIDTH * 3]; //fill after process
    frame.data_for_mat = new float [DISPLAY_RESOLUTION_HEIGHT
        * DISPLAY_RESOLUTION_WIDTH * 3];
    process_and_pad_image(frame.data_for_mat, image_uchar,
                          DISPLAY_RESOLUTION_WIDTH, DISPLAY_RESOLUTION_HEIGHT,
                          0);

    frame.scale = scale;
    //pad and transform to float
    int offset = 3 * NET_RESOLUTION_HEIGHT * NET_RESOLUTION_WIDTH;
    frame.data = new float [BATCH_SIZE * offset];
    int target_width, target_height;
    cv::Mat image_temp;
    //LOG(ERROR) << "frame.index: " << frame.index;
    for(int i=0; i < BATCH_SIZE; i++) {
      float scale = START_SCALE - i*SCALE_GAP;
      target_width = 16 * ceil(NET_RESOLUTION_WIDTH * scale /16);
      target_height = 16 * ceil(NET_RESOLUTION_HEIGHT * scale /16);

      CHECK_LE(target_width, NET_RESOLUTION_WIDTH);
      CHECK_LE(target_height, NET_RESOLUTION_HEIGHT);

      resize(image_uchar, image_temp, cv::Size(target_width, target_height),
             0, 0, CV_INTER_AREA);
      process_and_pad_image(frame.data + i * offset, image_temp,
                            NET_RESOLUTION_WIDTH, NET_RESOLUTION_HEIGHT, 1);
    }
    frame.commit_time = get_wall_time();
    frame.preprocessed_time = get_wall_time();

    global.input_queue.push(frame);
    global.input_queue_cond.notify_one();

    // If we reach the end of a video, loop
    if (frame_counter >= global.image_list.size()) {
      LOG(INFO) << "Done, exiting. # frames: " << frame_counter;
      // Wait until the queues are clear before exiting
      while (global.input_queue.size()
             || global.output_queue.size()
             || global.output_queue_ordered.size()) {
        // Should actually wait until they finish writing to disk
        // This could exit before the last frame is written.
        usleep(1000*1000.0);
        continue;
      }
      global.quit_threads = true;
      global.uistate.is_video_paused = true;
    }
  }
  return nullptr;
}

void* getFrameFromCam(void *i)
{
  cv::VideoCapture cap;
  double target_frame_time = 0;
  double target_frame_rate = 0;
  if (!_image_dir.empty()) {
    return getFrameFromDir(i);
  }

  if (_video.empty()) {
    CHECK(cap.open(_camera_index)) << "Couldn't open camera " << _camera_index;
    cap.set(CV_CAP_PROP_FRAME_WIDTH,CAMERA_FRAME_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,CAMERA_FRAME_HEIGHT);
  } else {
    CHECK(cap.open(_video)) << "Couldn't open video file " << _video;
    target_frame_rate = cap.get(CV_CAP_PROP_FPS);
    target_frame_time = 1.0/target_frame_rate;
    if (_start_frame) {
      cap.set(CV_CAP_PROP_POS_FRAMES, _start_frame);
    }
  }

  int global_counter = 1;
  int frame_counter = 0;
  cv::Mat image_uchar;
  cv::Mat image_uchar_orig;
  cv::Mat image_uchar_prev;
  double last_frame_time = -1;
  while(1) {
    if (global.quit_threads or !ros::ok()) {
      break;
    }
    if (!_video.empty() && _no_frame_drops) {
      // If the queue is too long, wait for a bit
      if (global.input_queue.size()>10) {
        usleep(10*1000.0);
        continue;
      }
    }
    cap >> image_uchar_orig;
    // Keep a count of how many frames we've seen in the video
    if (!_video.empty()) {
      if (global.uistate.seek_to_frame!=-1) {
        cap.set(CV_CAP_PROP_POS_FRAMES, global.uistate.current_frame);
        global.uistate.seek_to_frame = -1;
      }
      frame_counter = cap.get(CV_CAP_PROP_POS_FRAMES);

      VLOG(3) << "Frame: " << frame_counter << " / " <<
                 cap.get(CV_CAP_PROP_FRAME_COUNT);
      // This should probably be protected.
      global.uistate.current_frame = frame_counter-1;
      if (global.uistate.is_video_paused) {
        cap.set(CV_CAP_PROP_POS_FRAMES, frame_counter-1);
        frame_counter -= 1;
      }

      // Sleep to get the right frame rate.
      double cur_frame_time = get_wall_time();
      double interval = (cur_frame_time-last_frame_time);
      VLOG(3) << "cur_frame_time " << (cur_frame_time);
      VLOG(3) << "last_frame_time " << (last_frame_time);
      VLOG(3) << "cur-last_frame_time " << (cur_frame_time - last_frame_time);
      VLOG(3) << "Video target frametime " << 1.0/target_frame_time
              << " read frametime " << 1.0/interval;
      if (interval<target_frame_time) {
        VLOG(3) << "Sleeping for " << (target_frame_time-interval)*1000.0;
        usleep((target_frame_time-interval)*1000.0*1000.0);
        cur_frame_time = get_wall_time();
      }
      last_frame_time = cur_frame_time;
    } else {
      // From camera, just increase counter.
      if (global.uistate.is_video_paused) {
        image_uchar_orig = image_uchar_prev;
      }
      image_uchar_prev = image_uchar_orig;
      frame_counter++;
    }

    // TODO: The entire scaling code should be rewritten and better matched
    // to the imresize_layer. Confusingly, for the demo, there's an intermediate
    // display resolution to which the original image is resized.
    double scale = 0;
    if (image_uchar_orig.cols/(double)image_uchar_orig.rows>
        DISPLAY_RESOLUTION_WIDTH/(double)DISPLAY_RESOLUTION_HEIGHT) {
      scale = DISPLAY_RESOLUTION_WIDTH/(double)image_uchar_orig.cols;
    } else {
      scale = DISPLAY_RESOLUTION_HEIGHT/(double)image_uchar_orig.rows;
    }
    VLOG(4) << "Scale to DISPLAY_RESOLUTION_WIDTH/HEIGHT: " << scale;
    cv::Mat M = cv::Mat::eye(2,3,CV_64F);
    M.at<double>(0,0) = scale;
    M.at<double>(1,1) = scale;
    warpAffine(image_uchar_orig, image_uchar, M,
               cv::Size(DISPLAY_RESOLUTION_WIDTH, DISPLAY_RESOLUTION_HEIGHT),
               CV_INTER_CUBIC,
               cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
    // resize(image_uchar, image_uchar, Size(new_width, new_height), 0, 0,
    // CV_INTER_CUBIC);
    image_uchar_prev = image_uchar_orig;

    if ( image_uchar.empty() )
      continue;

    Frame frame;
    frame.scale = scale;
    frame.index = global_counter++;
    frame.video_frame_number = global.uistate.current_frame;
    frame.data_for_wrap = new unsigned char [DISPLAY_RESOLUTION_HEIGHT
        * DISPLAY_RESOLUTION_WIDTH * 3]; //fill after process
    frame.data_for_mat = new float [DISPLAY_RESOLUTION_HEIGHT
        * DISPLAY_RESOLUTION_WIDTH * 3];
    process_and_pad_image(frame.data_for_mat, image_uchar,
                          DISPLAY_RESOLUTION_WIDTH, DISPLAY_RESOLUTION_HEIGHT,
                          0);

    //pad and transform to float
    int offset = 3 * NET_RESOLUTION_HEIGHT * NET_RESOLUTION_WIDTH;
    frame.data = new float [BATCH_SIZE * offset];
    int target_width;
    int target_height;
    cv::Mat image_temp;
    for(int i=0; i < BATCH_SIZE; i++) {
      float scale = START_SCALE - i*SCALE_GAP;
      target_width = 16 * ceil(NET_RESOLUTION_WIDTH * scale /16);
      target_height = 16 * ceil(NET_RESOLUTION_HEIGHT * scale /16);

      CHECK_LE(target_width, NET_RESOLUTION_WIDTH);
      CHECK_LE(target_height, NET_RESOLUTION_HEIGHT);

      cv::resize(image_uchar, image_temp, cv::Size(target_width, target_height),
                 0, 0, CV_INTER_AREA);
      process_and_pad_image(frame.data + i * offset, image_temp,
                            NET_RESOLUTION_WIDTH, NET_RESOLUTION_HEIGHT, 1);
    }
    frame.commit_time = get_wall_time();
    frame.preprocessed_time = get_wall_time();

    global.input_queue.push(frame);

    // If we reach the end of a video, loop
    if (!_video.empty() && frame_counter >= cap.get(CV_CAP_PROP_FRAME_COUNT)) {
      if (!_write_frames.empty()) {
        LOG(INFO) << "Done, exiting. # frames: " << frame_counter;
        // This is the last frame (also the last emmitted frame)
        // Wait until the queues are clear before exiting
        while (global.input_queue.size()
               || global.output_queue.size()
               || global.output_queue_ordered.size()) {
          // Should actually wait until they finish writing to disk.
          // This could exit before the last frame is written.
          usleep(1000*1000.0);
          continue;
        }
        global.quit_threads = true;
        global.uistate.is_video_paused = true;
      } else {
        LOG(INFO) << "Looping video after " << cap.get(CV_CAP_PROP_FRAME_COUNT)
                  << " frames";
        cap.set(CV_CAP_PROP_POS_FRAMES, 0);
      }
    }
  }
  return nullptr;
}

void* getFrames(void *i) {
  if (_rgb_topic.empty())
  {
    getFrameFromCam(i);
  }
  else
  {
    int global_counter = 1;
    cv::Mat mat_from_frame, image_uchar;
    double scale;
    while(1)
    {

      ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Wait");
      std::unique_lock<std::mutex> lk(global.get_frames_mutex);
      global.get_frames_cond.wait(lk, []{return
            global.input_queue.size() < _max_size_input_queue;});
      ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Gooo");
      if (global.quit_threads or !ros::ok())
        break;
      {
        ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Wait CBK");
        std::lock_guard<std::mutex> lock(_callback_mutex);
        ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Locked CBK");
        if (global.quit_threads or !ros::ok())
          break;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(_rgb_frame_from_sensor,
                                       sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          break;
        }
        mat_from_frame = cv_ptr->image;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(_depth_frame_from_sensor,
                                       sensor_msgs::image_encodings::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          break;
        }
        _depth_image = cv_ptr->image;
        _fx = _camera_info.K.at(0);
        _fy = _camera_info.K.at(4);
        _constant_x = 1.0f / _fx;
        _constant_y = 1.0f / _fy;
        _cx = _camera_info.K.at(2);
        _cy = _camera_info.K.at(5);
        if (mat_from_frame.cols/(double)mat_from_frame.rows>
            DISPLAY_RESOLUTION_WIDTH/(double)DISPLAY_RESOLUTION_HEIGHT) {
          scale = DISPLAY_RESOLUTION_WIDTH/(double)mat_from_frame.cols;
        } else {
          scale = DISPLAY_RESOLUTION_HEIGHT/(double)mat_from_frame.rows;
        }
        VLOG(4) << "Scale to DISPLAY_RESOLUTION_WIDTH/HEIGHT: " << scale;
        cv::Mat M = cv::Mat::eye(2,3,CV_64F);
        M.at<double>(0,0) = scale;
        M.at<double>(1,1) = scale;
        warpAffine(mat_from_frame, image_uchar, M,
                   cv::Size(DISPLAY_RESOLUTION_WIDTH,
                            DISPLAY_RESOLUTION_HEIGHT),
                   CV_INTER_CUBIC,
                   cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
        warpAffine(_depth_image, _depth_image, M,
                   cv::Size(DISPLAY_RESOLUTION_WIDTH,
                            DISPLAY_RESOLUTION_HEIGHT),
                   CV_INTER_CUBIC,
                   cv::BORDER_CONSTANT, ushort(0));
        ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Unlock CBK");
      }
      if ( image_uchar.empty() )
      {
        continue;
      }
      Frame frame;
      frame.scale = scale;
      frame.index = global_counter++;
      frame.video_frame_number = global.uistate.current_frame;
      frame.data_for_wrap = new unsigned char [DISPLAY_RESOLUTION_HEIGHT
          * DISPLAY_RESOLUTION_WIDTH * 3]; //fill after process
      frame.data_for_mat = new float [DISPLAY_RESOLUTION_HEIGHT
          * DISPLAY_RESOLUTION_WIDTH * 3];
      process_and_pad_image(frame.data_for_mat, image_uchar,
                            DISPLAY_RESOLUTION_WIDTH, DISPLAY_RESOLUTION_HEIGHT,
                            0);

      //pad and transform to float
      int offset = 3 * NET_RESOLUTION_HEIGHT * NET_RESOLUTION_WIDTH;
      frame.data = new float [BATCH_SIZE * offset];
      int target_width;
      int target_height;
      cv::Mat image_temp;
      for(int i=0; i < BATCH_SIZE; i++) {
        float scale = START_SCALE - i*SCALE_GAP;
        target_width = 16 * ceil(NET_RESOLUTION_WIDTH * scale /16);
        target_height = 16 * ceil(NET_RESOLUTION_HEIGHT * scale /16);

        CHECK_LE(target_width, NET_RESOLUTION_WIDTH);
        CHECK_LE(target_height, NET_RESOLUTION_HEIGHT);

        cv::resize(image_uchar, image_temp, cv::Size(target_width,
                                                     target_height), 0, 0,
                   CV_INTER_AREA);
        process_and_pad_image(frame.data + i * offset, image_temp,
                              NET_RESOLUTION_WIDTH, NET_RESOLUTION_HEIGHT, 1);
      }
      frame.commit_time = get_wall_time();
      frame.preprocessed_time = get_wall_time();
      {
        std::lock_guard<std::recursive_mutex> lock(_depth_queue_mutex);
        if (global.quit_threads or !ros::ok())
          break;
        _depth_queue[frame.index] = _depth_image.clone();
        _header_queue[frame.index] = _rgb_frame_from_sensor.header;
      }
      global.input_queue.push(frame);
      global.input_queue_mutex.unlock();
      ROS_INFO_STREAM_COND(DEBUG > 1, "GETFRAME: Notify PRCFRAME");
      global.input_queue_cond.notify_one();
    }
  }

  return nullptr;
}

int connectLimbs(
    std::vector< std::vector<double>> &subset,
    std::vector< std::vector< std::vector<double> > > &connection,
    const float *heatmap_pointer,
    const float *peaks,
    int max_peaks,
    float *joints,
    ModelDescriptor *model_descriptor) {

  const auto num_parts = model_descriptor->get_number_parts();
  const auto limbSeq = model_descriptor->get_limb_sequence();
  const auto mapIdx = model_descriptor->get_map_idx();
  const auto number_limb_seq = model_descriptor->number_limb_sequence();

  int SUBSET_CNT = num_parts+2;
  int SUBSET_SCORE = num_parts+1;
  int SUBSET_SIZE = num_parts+3;

  CHECK_EQ(num_parts, 15);
  CHECK_EQ(number_limb_seq, 14);

  int peaks_offset = 3*(max_peaks+1);
  subset.clear();
  connection.clear();

  for(int k = 0; k < number_limb_seq; k++) {
    const float* map_x = heatmap_pointer + mapIdx[2*k] * NET_RESOLUTION_HEIGHT
        * NET_RESOLUTION_WIDTH;
    const float* map_y = heatmap_pointer + mapIdx[2*k+1] * NET_RESOLUTION_HEIGHT
        * NET_RESOLUTION_WIDTH;

    const float* candA = peaks + limbSeq[2*k]*peaks_offset;
    const float* candB = peaks + limbSeq[2*k+1]*peaks_offset;

    std::vector< std::vector<double> > connection_k;
    int nA = candA[0];
    int nB = candB[0];

    // add parts into the subset in special case
    if (nA ==0 && nB ==0) {
      continue;
    }
    else if (nA ==0) {
      for(int i = 1; i <= nB; i++) {
        std::vector<double> row_vec(SUBSET_SIZE, 0);
        row_vec[ limbSeq[2*k+1] ] = limbSeq[2*k+1]*peaks_offset + i*3 + 2;
        row_vec[SUBSET_CNT] = 1;  //last number in each row is the parts number
        //of that person
        row_vec[SUBSET_SCORE] = candB[i*3+2]; //second last number in each row
        //is the total score
        subset.push_back(row_vec);
      }
      continue;
    }
    else if (nB ==0) {
      for(int i = 1; i <= nA; i++) {
        std::vector<double> row_vec(SUBSET_SIZE, 0);
        row_vec[ limbSeq[2*k] ] = limbSeq[2*k]*peaks_offset + i*3 + 2;//store
        //the index
        row_vec[SUBSET_CNT] = 1;  //last number in each row is the parts
        //number of that person
        row_vec[SUBSET_SCORE] = candA[i*3+2]; //second last number in each
        //row is the total score
        subset.push_back(row_vec);
      }
      continue;
    }

    std::vector< std::vector<double>> temp;
    const int num_inter = 10;

    for(int i = 1; i <= nA; i++) {
      for(int j = 1; j <= nB; j++) {
        float s_x = candA[i*3];
        float s_y = candA[i*3+1];
        float d_x = candB[j*3] - candA[i*3];
        float d_y = candB[j*3+1] - candA[i*3+1];
        float norm_vec = sqrt( pow(d_x,2) + pow(d_y,2) );
        if (norm_vec<1e-6) {
          continue;
        }
        float vec_x = d_x/norm_vec;
        float vec_y = d_y/norm_vec;

        float sum = 0;
        int count = 0;

        for(int lm=0; lm < num_inter; lm++) {
          int my = round(s_y + lm*d_y/num_inter);
          int mx = round(s_x + lm*d_x/num_inter);
          int idx = my * NET_RESOLUTION_WIDTH + mx;
          float score = (vec_x*map_x[idx] + vec_y*map_y[idx]);
          if (score > global.connect_inter_threshold) {
            sum = sum + score;
            count ++;
          }
        }
        //float score = sum / count; // + std::min((130/dist-1),0.f)

        if (count > global.connect_inter_min_above_threshold) {//num_inter*0.8)
          // parts score + cpnnection score
          std::vector<double> row_vec(4, 0);
          row_vec[3] = sum/count + candA[i*3+2] + candB[j*3+2]; //score_all
          row_vec[2] = sum/count;
          row_vec[0] = i;
          row_vec[1] = j;
          temp.push_back(row_vec);
        }
      }
    }

    //** select the top num connection, assuming that each part occur only once
    // sort rows in descending order based on parts + connection score
    if (temp.size() > 0)
      std::sort(temp.begin(), temp.end(), ColumnCompare());

    int num = std::min(nA, nB);
    int cnt = 0;
    std::vector<int> occurA(nA, 0);
    std::vector<int> occurB(nB, 0);

    for(int row =0; row < temp.size(); row++) {
      if (cnt==num) {
        break;
      }
      else{
        int i = int(temp[row][0]);
        int j = int(temp[row][1]);
        float score = temp[row][2];
        if ( occurA[i-1] == 0 && occurB[j-1] == 0 ) { // && score> (1+thre)
          std::vector<double> row_vec(3, 0);
          row_vec[0] = limbSeq[2*k]*peaks_offset + i*3 + 2;
          row_vec[1] = limbSeq[2*k+1]*peaks_offset + j*3 + 2;
          row_vec[2] = score;
          connection_k.push_back(row_vec);
          cnt = cnt+1;
          occurA[i-1] = 1;
          occurB[j-1] = 1;
        }
      }
    }

    if (k==0) {
      std::vector<double> row_vec(num_parts+3, 0);
      for(int i = 0; i < connection_k.size(); i++) {
        double indexA = connection_k[i][0];
        double indexB = connection_k[i][1];
        row_vec[limbSeq[0]] = indexA;
        row_vec[limbSeq[1]] = indexB;
        row_vec[SUBSET_CNT] = 2;
        // add the score of parts and the connection
        row_vec[SUBSET_SCORE] = peaks[int(indexA)] + peaks[int(indexB)]
            + connection_k[i][2];
        subset.push_back(row_vec);
      }
    }
    else{
      if (connection_k.size()==0) {
        continue;
      }
      // A is already in the subset, find its connection B
      for(int i = 0; i < connection_k.size(); i++) {
        int num = 0;
        double indexA = connection_k[i][0];
        double indexB = connection_k[i][1];

        for(int j = 0; j < subset.size(); j++) {
          if (subset[j][limbSeq[2*k]] == indexA) {
            subset[j][limbSeq[2*k+1]] = indexB;
            num = num+1;
            subset[j][SUBSET_CNT] = subset[j][SUBSET_CNT] + 1;
            subset[j][SUBSET_SCORE] = subset[j][SUBSET_SCORE]
                + peaks[int(indexB)] + connection_k[i][2];
          }
        }
        // if can not find partA in the subset, create a new subset
        if (num==0) {
          std::vector<double> row_vec(SUBSET_SIZE, 0);
          row_vec[limbSeq[2*k]] = indexA;
          row_vec[limbSeq[2*k+1]] = indexB;
          row_vec[SUBSET_CNT] = 2;
          row_vec[SUBSET_SCORE] = peaks[int(indexA)] + peaks[int(indexB)]
              + connection_k[i][2];
          subset.push_back(row_vec);
        }
      }
    }
  }

  //** joints by deleting some rows of subset which has few parts occur
  int cnt = 0;
  for(int i = 0; i < subset.size(); i++) {
    if (subset[i][SUBSET_CNT]>=global.connect_min_subset_cnt &&
        (subset[i][SUBSET_SCORE]/subset[i][SUBSET_CNT])>
        global.connect_min_subset_score) {
      for(int j = 0; j < num_parts; j++) {
        int idx = int(subset[i][j]);
        if (idx) {
          joints[cnt*num_parts*3 + j*3 +2] = peaks[idx];
          joints[cnt*num_parts*3 + j*3 +1] = peaks[idx-1]
              * DISPLAY_RESOLUTION_HEIGHT/ (float)NET_RESOLUTION_HEIGHT;
          joints[cnt*num_parts*3 + j*3] = peaks[idx-2]
              * DISPLAY_RESOLUTION_WIDTH/ (float)NET_RESOLUTION_WIDTH;
        }
        else{
          joints[cnt*num_parts*3 + j*3 +2] = 0;
          joints[cnt*num_parts*3 + j*3 +1] = 0;
          joints[cnt*num_parts*3 + j*3] = 0;
        }
      }
      cnt++;
      if (cnt==MAX_PEOPLE) break;
    }
  }

  return cnt;
}

int distanceThresholdPeaks(const float *in_peaks, int max_peaks,
                           float *peaks, ModelDescriptor *model_descriptor) {
  // Post-process peaks to remove those which are within sqrt(dist_threshold2)
  // of each other.

  const auto num_parts = model_descriptor->get_number_parts();
  const float dist_threshold2 = 6*6;
  int peaks_offset = 3*(max_peaks+1);

  int total_peaks = 0;
  for(int p = 0; p < num_parts; p++) {
    const float *pipeaks = in_peaks + p*peaks_offset;
    float *popeaks = peaks + p*peaks_offset;
    int num_in_peaks = int(pipeaks[0]);
    int num_out_peaks = 0; // Actual number of peak count
    for (int c1=0;c1<num_in_peaks;c1++) {
      float x1 = pipeaks[(c1+1)*3+0];
      float y1 = pipeaks[(c1+1)*3+1];
      float s1 = pipeaks[(c1+1)*3+2];
      bool keep = true;
      for (int c2=0;c2<num_out_peaks;c2++) {
        float x2 = popeaks[(c2+1)*3+0];
        float y2 = popeaks[(c2+1)*3+1];
        float s2 = popeaks[(c2+1)*3+2];
        float dist2 = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
        if (dist2<dist_threshold2) {
          // This peak is too close to a peak already in the output buffer
          // so don't add it.
          keep = false;
          if (s1>s2) {
            // It's better than the one in the output buffer
            // so we swap it.
            popeaks[(c2+1)*3+0] = x1;
            popeaks[(c2+1)*3+1] = y1;
            popeaks[(c2+1)*3+2] = s1;
          }
        }
      }
      if (keep && num_out_peaks<max_peaks) {
        // We don't already have a better peak within the threshold distance
        popeaks[(num_out_peaks+1)*3+0] = x1;
        popeaks[(num_out_peaks+1)*3+1] = y1;
        popeaks[(num_out_peaks+1)*3+2] = s1;
        num_out_peaks++;
      }
    }
    // if (num_in_peaks!=num_out_peaks) {
    //LOG(INFO) << "Part: " << p << " in peaks: "<< num_in_peaks << " out: "
    //<< num_out_peaks;
    // }
    popeaks[0] = float(num_out_peaks);
    total_peaks += num_out_peaks;
  }
  return total_peaks;
}

int connectLimbsCOCO(
    std::vector< std::vector<double>> &subset,
    std::vector< std::vector< std::vector<double> > > &connection,
    const float *heatmap_pointer,
    const float *in_peaks,
    int max_peaks,
    float *joints,
    ModelDescriptor *model_descriptor) {
  /* Parts Connection ---------------------------------------*/
  const auto num_parts = model_descriptor->get_number_parts();
  const auto limbSeq = model_descriptor->get_limb_sequence();
  const auto mapIdx = model_descriptor->get_map_idx();
  const auto number_limb_seq = model_descriptor->number_limb_sequence();

  CHECK_EQ(num_parts, 18) << "Wrong connection function for model";
  CHECK_EQ(number_limb_seq, 19) << "Wrong connection function for model";

  int SUBSET_CNT = num_parts+2;
  int SUBSET_SCORE = num_parts+1;
  int SUBSET_SIZE = num_parts+3;

  const int peaks_offset = 3*(max_peaks+1);

  const float *peaks = in_peaks;
  subset.clear();
  connection.clear();

  for(int k = 0; k < number_limb_seq; k++) {
    const float* map_x = heatmap_pointer + mapIdx[2*k] * NET_RESOLUTION_HEIGHT
        * NET_RESOLUTION_WIDTH;
    const float* map_y = heatmap_pointer + mapIdx[2*k+1] * NET_RESOLUTION_HEIGHT
        * NET_RESOLUTION_WIDTH;

    const float* candA = peaks + limbSeq[2*k]*peaks_offset;
    const float* candB = peaks + limbSeq[2*k+1]*peaks_offset;

    std::vector< std::vector<double> > connection_k;
    int nA = candA[0];
    int nB = candB[0];

    // add parts into the subset in special case
    if (nA ==0 && nB ==0) {
      continue;
    } else if (nA ==0) {
      for(int i = 1; i <= nB; i++) {
        int num = 0;
        int indexB = limbSeq[2*k+1];
        for(int j = 0; j < subset.size(); j++) {
          int off = limbSeq[2*k+1]*peaks_offset + i*3 + 2;
          if (subset[j][indexB] == off) {
            num = num+1;
            continue;
          }
        }
        if (num!=0) {
          //LOG(INFO) << " else if (nA==0) shouldn't have any nB
          // already assigned?";
        } else {
          std::vector<double> row_vec(SUBSET_SIZE, 0);
          row_vec[ limbSeq[2*k+1] ] = limbSeq[2*k+1]*peaks_offset + i*3 + 2;
          row_vec[SUBSET_CNT] = 1;  //last number in each row is the parts
          //number of that person
          row_vec[SUBSET_SCORE] = candB[i*3+2]; //second last number in
          //each row is the total score
          subset.push_back(row_vec);
        }
        //LOG(INFO) << "nA==0 New subset on part " << k << " subsets: "
        //<< subset.size();
      }
      continue;
    } else if (nB ==0) {
      for(int i = 1; i <= nA; i++) {
        int num = 0;
        int indexA = limbSeq[2*k];
        for(int j = 0; j < subset.size(); j++) {
          int off = limbSeq[2*k]*peaks_offset + i*3 + 2;
          if (subset[j][indexA] == off) {
            num = num+1;
            continue;
          }
        }
        if (num==0) {
          std::vector<double> row_vec(SUBSET_SIZE, 0);
          row_vec[ limbSeq[2*k] ] = limbSeq[2*k]*peaks_offset + i*3 + 2;
          row_vec[SUBSET_CNT] = 1;  // last number in each row is the parts
          // number of that person
          row_vec[SUBSET_SCORE] = candA[i*3+2]; // second last number in each
          // row is the total score
          subset.push_back(row_vec);
          //LOG(INFO) << "nB==0 New subset on part " << k << " subsets: "
          // << subset.size();
        } else {
          //LOG(INFO) << "nB==0 discarded would have added";
        }
      }
      continue;
    }

    std::vector< std::vector<double>> temp;
    const int num_inter = 10;

    for(int i = 1; i <= nA; i++) {
      for(int j = 1; j <= nB; j++) {
        float s_x = candA[i*3];
        float s_y = candA[i*3+1];
        float d_x = candB[j*3] - candA[i*3];
        float d_y = candB[j*3+1] - candA[i*3+1];
        float norm_vec = sqrt( d_x*d_x + d_y*d_y );
        if (norm_vec<1e-6) {
          // The peaks are coincident. Don't connect them.
          continue;
        }
        float vec_x = d_x/norm_vec;
        float vec_y = d_y/norm_vec;

        float sum = 0;
        int count = 0;

        for(int lm=0; lm < num_inter; lm++) {
          int my = round(s_y + lm*d_y/num_inter);
          int mx = round(s_x + lm*d_x/num_inter);
          if (mx>=NET_RESOLUTION_WIDTH) {
            //LOG(ERROR) << "mx " << mx << "out of range";
            mx = NET_RESOLUTION_WIDTH-1;
          }
          if (my>=NET_RESOLUTION_HEIGHT) {
            //LOG(ERROR) << "my " << my << "out of range";
            my = NET_RESOLUTION_HEIGHT-1;
          }
          CHECK_GE(mx,0);
          CHECK_GE(my,0);
          int idx = my * NET_RESOLUTION_WIDTH + mx;
          float score = (vec_x*map_x[idx] + vec_y*map_y[idx]);
          if (score > global.connect_inter_threshold) {
            sum = sum + score;
            count ++;
          }
        }
        //float score = sum / count; // + std::min((130/dist-1),0.f)

        if (count > global.connect_inter_min_above_threshold) {//num_inter*0.8)
          // parts score + cpnnection score
          std::vector<double> row_vec(4, 0);
          row_vec[3] = sum/count + candA[i*3+2] + candB[j*3+2]; //score_all
          row_vec[2] = sum/count;
          row_vec[0] = i;
          row_vec[1] = j;
          temp.push_back(row_vec);
        }
      }
    }

    //** select the top num connection, assuming that each part occur only once
    // sort rows in descending order based on parts + connection score
    if (temp.size() > 0)
      std::sort(temp.begin(), temp.end(), ColumnCompare());

    int num = std::min(nA, nB);
    int cnt = 0;
    std::vector<int> occurA(nA, 0);
    std::vector<int> occurB(nB, 0);

    for(int row =0; row < temp.size(); row++) {
      if (cnt==num) {
        break;
      }
      else{
        int i = int(temp[row][0]);
        int j = int(temp[row][1]);
        float score = temp[row][2];
        if ( occurA[i-1] == 0 && occurB[j-1] == 0 ) { // && score> (1+thre)
          std::vector<double> row_vec(3, 0);
          row_vec[0] = limbSeq[2*k]*peaks_offset + i*3 + 2;
          row_vec[1] = limbSeq[2*k+1]*peaks_offset + j*3 + 2;
          row_vec[2] = score;
          connection_k.push_back(row_vec);
          cnt = cnt+1;
          occurA[i-1] = 1;
          occurB[j-1] = 1;
        }
      }
    }

    //** cluster all the joints candidates into subset based on the part
    // connection
    // initialize first body part connection 15&16
    if (k==0) {
      std::vector<double> row_vec(num_parts+3, 0);
      for(int i = 0; i < connection_k.size(); i++) {
        double indexB = connection_k[i][1];
        double indexA = connection_k[i][0];
        row_vec[limbSeq[0]] = indexA;
        row_vec[limbSeq[1]] = indexB;
        row_vec[SUBSET_CNT] = 2;
        // add the score of parts and the connection
        row_vec[SUBSET_SCORE] = peaks[int(indexA)]
            + peaks[int(indexB)] + connection_k[i][2];
        subset.push_back(row_vec);
      }
    } else{
      if (connection_k.size()==0) {
        continue;
      }

      // A is already in the subset, find its connection B
      for(int i = 0; i < connection_k.size(); i++) {
        int num = 0;
        double indexA = connection_k[i][0];
        double indexB = connection_k[i][1];

        for(int j = 0; j < subset.size(); j++) {
          if (subset[j][limbSeq[2*k]] == indexA) {
            subset[j][limbSeq[2*k+1]] = indexB;
            num = num+1;
            subset[j][SUBSET_CNT] = subset[j][SUBSET_CNT] + 1;
            subset[j][SUBSET_SCORE] = subset[j][SUBSET_SCORE]
                + peaks[int(indexB)] + connection_k[i][2];
          }
        }
        // if can not find partA in the subset, create a new subset
        if (num==0) {
          std::vector<double> row_vec(SUBSET_SIZE, 0);
          row_vec[limbSeq[2*k]] = indexA;
          row_vec[limbSeq[2*k+1]] = indexB;
          row_vec[SUBSET_CNT] = 2;
          row_vec[SUBSET_SCORE] = peaks[int(indexA)] + peaks[int(indexB)]
              + connection_k[i][2];
          subset.push_back(row_vec);
        }
      }
    }
  }

  //** joints by deleteing some rows of subset which has few parts occur
  int cnt = 0;
  for(int i = 0; i < subset.size(); i++) {
    if (subset[i][SUBSET_CNT]<1) {
      LOG(INFO) << "BAD SUBSET_CNT";
    }
    if (subset[i][SUBSET_CNT]>=global.connect_min_subset_cnt &&
        (subset[i][SUBSET_SCORE]/subset[i][SUBSET_CNT])>
        global.connect_min_subset_score) {
      for(int j = 0; j < num_parts; j++) {
        int idx = int(subset[i][j]);
        if (idx) {
          joints[cnt*num_parts*3 + j*3 +2] = peaks[idx];
          joints[cnt*num_parts*3 + j*3 +1] = peaks[idx-1]*
              DISPLAY_RESOLUTION_HEIGHT/ (float)NET_RESOLUTION_HEIGHT;
          joints[cnt*num_parts*3 + j*3] = peaks[idx-2]*
              DISPLAY_RESOLUTION_WIDTH/ (float)NET_RESOLUTION_WIDTH;
        }
        else{
          joints[cnt*num_parts*3 + j*3 +2] = 0;
          joints[cnt*num_parts*3 + j*3 +1] = 0;
          joints[cnt*num_parts*3 + j*3] = 0;
        }
      }
      cnt++;
      if (cnt==MAX_PEOPLE) break;
    }
  }

  return cnt;
}


void* processFrame(void *i) {
  int tid = *((int *) i);
  warmup(tid);
  LOG(INFO) << "GPU " << tid << " is ready";
  Frame frame;

  int offset = NET_RESOLUTION_WIDTH * NET_RESOLUTION_HEIGHT * 3;
  //bool empty = false;

  Frame frame_batch[BATCH_SIZE];

  std::vector< std::vector<double>> subset;
  std::vector< std::vector< std::vector<double> > > connection;

  const boost::shared_ptr<caffe::Blob<float>> heatmap_blob =
      net_copies[tid].person_net->blob_by_name("resized_map");
  const boost::shared_ptr<caffe::Blob<float>> joints_blob =
      net_copies[tid].person_net->blob_by_name("joints");

  caffe::NmsLayer<float> *nms_layer =
      (caffe::NmsLayer<float>*)net_copies[tid].person_net->
      layer_by_name("nms").get();

  //while(!empty) {
  while(1) {
    //LOG(ERROR) << "start";
    int valid_data = 0;
    //for(int n = 0; n < BATCH_SIZE; n++) {
    ROS_INFO_STREAM_COND(DEBUG > 1, "PRCFRAME: Wait");
    std::unique_lock<std::mutex> lk(global.input_queue_mutex);
    global.input_queue_cond.wait(lk, [](){return
          global.input_queue.size() > 0;});
    ROS_INFO_STREAM_COND(DEBUG > 1, "PRCFRAME: Gooo");
    while(valid_data<1 && global.input_queue.size() > 0) {
      if (global.quit_threads or !ros::ok())
        break;
      frame = global.input_queue.pop();
      global.get_frames_cond.notify_one();
      //consider dropping it
      frame.gpu_fetched_time = get_wall_time();
      double elaspsed_time = frame.gpu_fetched_time - frame.commit_time;
      if (elaspsed_time > 0.1
          && !_no_frame_drops) {//0.1*BATCH_SIZE) { //0.1*BATCH_SIZE
        //drop frame
        VLOG(1) << "skip frame " << frame.index;
        delete [] frame.data;
        delete [] frame.data_for_mat;
        delete [] frame.data_for_wrap;
        {
          ROS_INFO_STREAM_COND(DEBUG > 1, "PRCFRAME: Wait depthQueue");
          std::lock_guard<std::recursive_mutex> lock(_depth_queue_mutex);
          _depth_queue.erase(frame.index);
          _header_queue.erase(frame.index);
          ROS_INFO_STREAM_COND(DEBUG > 1, "PRCFRAME: Release depthQueue");
        }
        //n--;

        const std::lock_guard<std::mutex> lock{global.mutex};
        global.dropped_index.push(frame.index);
        continue;
      }
      //double tic1  = get_wall_time();

      cudaMemcpy(net_copies[tid].canvas, frame.data_for_mat,
                 DISPLAY_RESOLUTION_WIDTH * DISPLAY_RESOLUTION_HEIGHT * 3 *
                 sizeof(float), cudaMemcpyHostToDevice);

      frame_batch[0] = frame;
      float* pointer =
          net_copies[tid].person_net->blobs()[0]->mutable_gpu_data();

      cudaMemcpy(pointer + 0 * offset, frame_batch[0].data, BATCH_SIZE *
          offset * sizeof(float), cudaMemcpyHostToDevice);
      valid_data++;
      //VLOG(2) << "Host->device " << (get_wall_time()-tic1)*1000.0 << " ms.";
    }
    if (valid_data == 0)
    {
      continue;
    }
    nms_layer->SetThreshold(global.nms_threshold);
    net_copies[tid].person_net->ForwardFrom(0);
    VLOG(2) << "CNN time " << (get_wall_time()-frame.gpu_fetched_time)*1000.0
            << " ms.";
    //cudaDeviceSynchronize();
    float* heatmap_pointer = heatmap_blob->mutable_cpu_data();
    const float* peaks = joints_blob->mutable_cpu_data();

    float joints[MAX_NUM_PARTS*3*MAX_PEOPLE]; //10*15*3

    int cnt = 0;
    // CHECK_EQ(net_copies[tid].nms_num_parts, 15);
    double tic = get_wall_time();
    const int num_parts = net_copies[tid].nms_num_parts;
    if (net_copies[tid].nms_num_parts==15) {
      cnt = connectLimbs(subset, connection,
                         heatmap_pointer, peaks,
                         net_copies[tid].nms_max_peaks, joints,
                         net_copies[tid].up_model_descriptor.get());
    } else {
      cnt = connectLimbsCOCO(subset, connection,
                             heatmap_pointer, peaks,
                             net_copies[tid].nms_max_peaks, joints,
                             net_copies[tid].up_model_descriptor.get());
    }

    VLOG(2) << "CNT: " << cnt << " Connect time "
            << (get_wall_time()-tic)*1000.0 << " ms.";
    net_copies[tid].num_people[0] = cnt;
    VLOG(2) << "num_people[i] = " << cnt;


    cudaMemcpy(net_copies[tid].joints, joints,
               MAX_NUM_PARTS*3*MAX_PEOPLE * sizeof(float),
               cudaMemcpyHostToDevice);

    if (subset.size() != 0) {
      //LOG(ERROR) << "Rendering";
      render(tid, heatmap_pointer); //only support batch size = 1!!!!
      for(int n = 0; n < valid_data and not(global.quit_threads
                                            or not ros::ok()); n++) {
        frame_batch[n].numPeople = net_copies[tid].num_people[n];
        frame_batch[n].gpu_computed_time = get_wall_time();
        frame_batch[n].joints = boost::shared_ptr<float[]>
            (new float[frame_batch[n].numPeople*MAX_NUM_PARTS*3]);
        for (int ij=0;ij<frame_batch[n].numPeople*num_parts*3 and not
             (global.quit_threads or not ros::ok());ij++) {
          frame_batch[n].joints[ij] = joints[ij];
        }


        cudaMemcpy(frame_batch[n].data_for_mat, net_copies[tid].canvas,
                   DISPLAY_RESOLUTION_HEIGHT * DISPLAY_RESOLUTION_WIDTH * 3 *
                   sizeof(float), cudaMemcpyDeviceToHost);
        global.output_queue.push(frame_batch[n]);
      }
    }
    else {
      render(tid, heatmap_pointer);
      //frame_batch[n].data should revert to 0-255
      for(int n = 0; n < valid_data and not (global.quit_threads
                                             or not ros::ok()); n++) {
        frame_batch[n].numPeople = 0;
        frame_batch[n].gpu_computed_time = get_wall_time();
        cudaMemcpy(frame_batch[n].data_for_mat, net_copies[tid].canvas,
                   DISPLAY_RESOLUTION_HEIGHT * DISPLAY_RESOLUTION_WIDTH * 3 *
                   sizeof(float), cudaMemcpyDeviceToHost);
        global.output_queue.push(frame_batch[n]);
      }
    }
    if(global.output_queue.size() > 0)
    {
      ROS_INFO_STREAM_COND(DEBUG > 1, "PRCFRAME: Notify PSTPROC");
      global.output_queue_cond.notify_one();
    }
  }
  return nullptr;
}

class FrameCompare{
public:
  bool operator() (const Frame &a, const Frame &b) const{
    return a.index > b.index;
  }
};

void* postProcessFrame(void *i) {
  //int tid = *((int *) i);
  Frame frame;

  while(1) {
    ROS_INFO_STREAM_COND(DEBUG > 1, "PSTPROC: Wait");
    std::unique_lock<std::mutex> lk(global.output_queue_mutex);
    global.output_queue_cond.wait(lk, [](){return
          global.output_queue.size() > 0;});
    ROS_INFO_STREAM_COND(DEBUG > 1, "PSTPROC: Gooo");
    if (global.quit_threads or !ros::ok())
      break;
    frame = global.output_queue.pop();
    frame.postprocesse_begin_time = get_wall_time();

    //Mat visualize(NET_RESOLUTION_HEIGHT, NET_RESOLUTION_WIDTH, CV_8UC3);

    int offset = DISPLAY_RESOLUTION_WIDTH * DISPLAY_RESOLUTION_HEIGHT;
    for(int c = 0; c < 3 and not (global.quit_threads or not ros::ok()); c++) {
      for(int i = 0; i < DISPLAY_RESOLUTION_HEIGHT and not
          (global.quit_threads or not ros::ok()); i++) {
        for(int j = 0; j < DISPLAY_RESOLUTION_WIDTH and not
            (global.quit_threads or not ros::ok()); j++) {
          int value =
              int(frame.data_for_mat[c*offset + i*DISPLAY_RESOLUTION_WIDTH + j]
              + 0.5);
          value = value<0 ? 0 : (value > 255 ? 255 : value);
          frame.data_for_wrap[3*(i*DISPLAY_RESOLUTION_WIDTH + j) + c] =
              (unsigned char)(value);
        }
      }
    }
    frame.postprocesse_end_time = get_wall_time();
    global.output_queue_mated.push(frame);
    ROS_INFO_STREAM_COND(DEBUG > 1, "PSTPROC: Notify BFRORDER");
    global.output_queue_mated_cond.notify_one();
  }
  return nullptr;
}

void* buffer_and_order(void* threadargs) { //only one thread can execute this
  FrameCompare comp;
  std::priority_queue<Frame, std::vector<Frame>, FrameCompare> buffer(comp);
  Frame frame;

  int sleep_time = 10 * 1000;

  int frame_waited = 1;
  while(1) {
    ROS_INFO_STREAM_COND(DEBUG > 1, "BFRORDER: Wait");
    std::unique_lock<std::mutex> lk(global.output_queue_mated_mutex);
    global.output_queue_mated_cond.wait(lk, [](){return
          global.output_queue_mated.size() > 0;});
    ROS_INFO_STREAM_COND(DEBUG > 1, "BFRORDER: Gooo");
    frame = global.output_queue_mated.pop();
    if (global.quit_threads or !ros::ok())
      break;
    frame.buffer_start_time = get_wall_time();
    VLOG(4) << "buffer getting " << frame.index << ", waiting for "
            << frame_waited;
    {
      std::lock_guard<std::mutex> lock{global.mutex};

      while(global.dropped_index.size()!=0 && global.dropped_index.top() ==
            frame_waited and not (global.quit_threads or not ros::ok())) {
        frame_waited++;
        global.dropped_index.pop();
      }
    }
    //LOG(ERROR) << "while end";

    if (frame.index == frame_waited) {  // if this is the frame we want,
      // just push it
      frame.buffer_end_time = get_wall_time();
      global.output_queue_ordered.push(frame);
      frame_waited++;
      while(buffer.size() != 0 && buffer.top().index == frame_waited and not
            (global.quit_threads or not ros::ok())) {
        Frame next = buffer.top();
        buffer.pop();
        next.buffer_end_time = get_wall_time();
        global.output_queue_ordered.push(next);
        frame_waited++;
      }
    }
    else {
      buffer.push(frame);
    }

    if (buffer.size() > BUFFER_SIZE) {
      //LOG(ERROR) << "buffer squeezed";
      Frame extra = buffer.top();
      buffer.pop();
      //LOG(ERROR) << "popping " << get<0>(extra);
      extra.buffer_end_time = get_wall_time();
      global.output_queue_ordered.push(extra);
      frame_waited = extra.index + 1;
      while(buffer.size() != 0 && buffer.top().index == frame_waited and not
            (global.quit_threads or not ros::ok())) {
        Frame next = buffer.top();
        buffer.pop();
        next.buffer_end_time = get_wall_time();
        global.output_queue_ordered.push(next);
        frame_waited++;
      }
    }
    if(global.output_queue_ordered.size() > 0)
    {
      ROS_INFO_STREAM_COND(DEBUG > 1, "BFRORDER: Notify DSPFRAME");
      global.output_queue_ordered_cond.notify_one();
    }
  }
  return nullptr;
}

rtpose_wrapper::Joint3DMsg
getJoint3D(double x, double y, double confidence, const cv::Mat& depth_frame)
{
  rtpose_wrapper::Joint3DMsg joint3D;
  int min_y = std::max<double>(y - _median_search_y_component,
                               0.0f);
  int min_x = std::max<double>(x - _median_search_x_component,
                               0.0f);
  int max_y = std::min<double>(y + _median_search_y_component,
                               depth_frame.rows);
  int max_x = std::min<double>(x + _median_search_x_component,
                               depth_frame.cols);
#if DEBUG > 0
  cv::rectangle(_colored_depth_image,
                cv::Point(min_x, min_y), cv::Point(max_x, max_y),
                cv::Scalar(0, 255, 255), 3);
  //  cv::ellipse(_colored_depth_image,cv::Point(x,y),
  //              cv::Size(_median_search_x_component,_median_search_y_component),0,
  //              0,0,cv::Scalar(0,255,255),5);
#endif
  cv::Rect rect(min_x, min_y, (max_x - min_x), (max_y - min_y));
  joint3D.z = medianBetweenValidPoints(depth_frame(rect))
      / 1000.0f;
  joint3D.x = (x - _cx) * joint3D.z * _constant_x;
  joint3D.y = (y - _cy) * joint3D.z * _constant_y;
  joint3D.confidence = confidence;
  joint3D.header = _depth_frame_from_sensor.header;
  joint3D.max_height = DISPLAY_RESOLUTION_HEIGHT;
  joint3D.max_width = DISPLAY_RESOLUTION_WIDTH;
  if (fabs(joint3D.z) < 0.001f
      or joint3D.confidence < _min_confidence_per_joint)
    joint3D.x = joint3D.y = joint3D.z =std::numeric_limits<double>::quiet_NaN();
  return joint3D;
}


bool
isValid(const Eigen::Vector3d& v)
{
  return not v.hasNaN();
}

void* displayFrame(void *i) { //single thread
  Frame frame;
  while(1) {
    if (global.quit_threads or !ros::ok())
      break;
    ROS_INFO_STREAM_COND(DEBUG > 1, "DSPFRAME: Wait");
    std::unique_lock<std::mutex> lk(global.output_queue_ordered_mutex);
    global.output_queue_ordered_cond.wait(lk, [](){return
          global.output_queue_ordered.size() > 0;});
    ROS_INFO_STREAM_COND(DEBUG > 1, "DSPFRAME: Gooo");
    frame = global.output_queue_ordered.pop();
    double tic = get_wall_time();
    cv::Mat wrap_frame(DISPLAY_RESOLUTION_HEIGHT, DISPLAY_RESOLUTION_WIDTH,
                       CV_8UC3, frame.data_for_wrap);
    cv::Mat frame_copy = wrap_frame.clone();
    cv::Mat depth_frame;
    std_msgs::Header header;
    {
      ROS_INFO_STREAM_COND(DEBUG > 1, "DSPFRAME: QueueWait");
      std::lock_guard<std::recursive_mutex> lock(_depth_queue_mutex);
      if (global.quit_threads or !ros::ok())
        break;
      depth_frame = _depth_queue[frame.index];
      header = _header_queue[frame.index];
      _depth_queue.erase(frame.index);
      _header_queue.erase(frame.index);
      ROS_INFO_STREAM_COND(DEBUG > 1, "DSPFRAME: QueueRelease");
    }
    //    if(!_no_display)
    //    {
#if DEBUG > 0
    depth_frame.convertTo(_colored_depth_image, CV_8UC1);
    //cv::equalizeHist(_colored_depth_image,_colored_depth_image);
    cv::cvtColor(_colored_depth_image, _colored_depth_image, CV_GRAY2RGB);
#endif
    //    }
    if (!_skeleton_topic_to_publish.empty() && !_rgb_topic.empty()) // publish
    {
      double scale = 1.0/frame.scale;
      const int num_parts =
          net_copies.at(0).up_model_descriptor->get_number_parts();
      char fname[256];

      //      visualization_msgs::MarkerArray skeleton_marker_array;
      rtpose_wrapper::SkeletonArrayMsg skeleton_array;
      skeleton_array.header = _depth_frame_from_sensor.header;
      skeleton_array.header.stamp =
          ros::Time::now();
      skeleton_array.rgb_header = header;
      skeleton_array.intrinsic_matrix.reserve(9);
      for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
          skeleton_array.intrinsic_matrix.push_back(_camera_info.K[i * 3 + j]);

      {
        for(int person = 0; person < frame.numPeople and not
            (global.quit_threads or not ros::ok()); ++person)
        {
          rtpose_wrapper::SkeletonMsg skeleton;
          skeleton.joints.resize(num_parts);
          for(int joint = 0; joint < 15 and not
              (global.quit_threads or not ros::ok()); ++joint)
          {
            // write confidences on rgb image for debug
            std::stringstream conf;
            conf << frame.joints[person * num_parts * 3 + joint * 3 + 2];
            cv::Scalar color(0,255,0);
            if (joint >= RSHOULDER and joint <= RWRIST or
                joint >= RHIP and joint <= RANKLE)
              color = cv::Scalar(180,0,0);
            if (frame.joints[person * num_parts * 3 + joint * 3 + 2] < _min_confidence_per_joint)
              color = cv::Scalar(0,0,255);
            cv::putText(wrap_frame, conf.str().substr(0,4),
                        cv::Point(scale *
                                  frame.joints[person * num_parts * 3 +
                        joint * 3 + 0] -3,
                        scale * frame.joints[person * num_parts * 3 +
                joint * 3 + 1] - 3),
                cv::FONT_HERSHEY_PLAIN, 1, color, 2);
            // 2d->3d
            skeleton.joints[joint] = getJoint3D(
                  scale * frame.joints[person * num_parts * 3 + joint * 3 + 0],
                scale * frame.joints[person * num_parts * 3 + joint * 3 + 1],
                frame.joints[person * num_parts * 3 + joint * 3 + 2],
                depth_frame);
#if DEBUG > 0
            if(joint == RELBOW or joint == RWRIST or joint == LWRIST or
               joint == LELBOW)
              cv::putText(_colored_depth_image, std::to_string(skeleton.joints[joint].z),
                          cv::Point(scale * frame.joints[person * num_parts * 3 + joint * 3 + 0] - _median_search_x_component + 2,
                          scale * frame.joints[person * num_parts * 3 + joint * 3 + 1] - _median_search_y_component+ 2),
                  cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
#endif
          }
          // head
          const rtpose_wrapper::Joint3DMsg& head = skeleton.joints[HEAD];
          const rtpose_wrapper::Joint3DMsg& neck = skeleton.joints[NECK];
          if(_head_joint_enhancement
             and std::isfinite(neck.x + neck.y + neck.z))
          {
            double x = ( (1 - _head_correction_param) *
                         frame.joints[person * num_parts * 3 + NECK * 3 + 0] +
                _head_correction_param * head.x);
            double y = ( (1 - _head_correction_param) *
                         frame.joints[person * num_parts * 3 + NECK * 3 + 1] +
                _head_correction_param * head.y);
            skeleton.joints[HEAD] = getJoint3D(x, y,
                                               head.confidence,
                                               depth_frame);
          }
          if(num_parts > 15)
            skeleton.skeleton_type = rtpose_wrapper::SkeletonMsg::COCO;
          else
            skeleton.skeleton_type = rtpose_wrapper::SkeletonMsg::MPI;

          // chest
          if(skeleton.skeleton_type == rtpose_wrapper::SkeletonMsg::COCO)
          {
            const Eigen::Vector3d rhip(skeleton.joints[RHIP].x,
                                       skeleton.joints[RHIP].y,
                                       skeleton.joints[RHIP].z);
            const Eigen::Vector3d lhip(skeleton.joints[LHIP].x,
                                       skeleton.joints[LHIP].y,
                                       skeleton.joints[LHIP].z);
            const Eigen::Vector3d neck(skeleton.joints[NECK].x,
                                       skeleton.joints[NECK].y,
                                       skeleton.joints[NECK].z);
            const Eigen::Vector3d rshoulder(skeleton.joints[RSHOULDER].x,
                                            skeleton.joints[RSHOULDER].y,
                                            skeleton.joints[RSHOULDER].z);
            const Eigen::Vector3d lshoulder(skeleton.joints[LSHOULDER].x,
                                            skeleton.joints[LSHOULDER].y,
                                            skeleton.joints[LSHOULDER].z);
            Eigen::Vector3d chest(std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::quiet_NaN());
            if(isValid(neck)
               and isValid(rhip)
               and isValid(lhip))
            {
              const Eigen::Vector3d q = (rhip + lhip) / 2;
              chest =
                  q + Eigen::Vector3d(neck - q).normalized() / 6;
              chest[2] = (q[2] + neck[2]) / 2; //z component should not be influenced
            }
            else if(isValid(rshoulder)
                    and isValid(lshoulder)
                    and isValid(rhip)
                    and isValid(lhip))
            {
              // weighted mean
              chest =
                  (rhip + lhip) * 0.4 + (lshoulder + rshoulder) * 0.1;
            }
            skeleton.joints[CHEST].x = chest[0];
            skeleton.joints[CHEST].y = chest[1];
            skeleton.joints[CHEST].z = chest[2];
          } // chest
          else // MPI
          {
            skeleton.joints[CHEST] =
                getJoint3D(frame.joints[person * num_parts * 3 + CHEST * 3 + 0],
                frame.joints[person * num_parts * 3 + CHEST * 3 + 1],
                frame.joints[person * num_parts * 3 + CHEST * 3 + 2],
                depth_frame);
          }

          //noise removal
          std::vector<double> valid_joint_depths;
          valid_joint_depths.reserve(skeleton.joints.size());
          for(auto& j: skeleton.joints)
          {
            if(isValid(Eigen::Vector3d(j.x, j.y, j.z)))
              valid_joint_depths.push_back(j.z);
          }
          double joint_median_depth = - _noise_threshold; // ensuring rejecting
          if(valid_joint_depths.size() > 0)
            joint_median_depth = medianBetweenValidPoints(valid_joint_depths);
          for(auto it = skeleton.joints.begin(), end = skeleton.joints.end();
              it != end; ++it)
          {
            if (fabs(it->z - joint_median_depth) > _noise_threshold){
              //ROS_INFO_STREAM("Current z: " << it->z << " median: " << joint_median_depth);
              it->x = it->y = it->z = std::numeric_limits<double>::quiet_NaN();
            }
          }
          // too few valid joints => reject skeleton
          auto valid_lambda = [](rtpose_wrapper::Joint3DMsg& j)
          {return isValid(Eigen::Vector3d(j.x, j.y, j.z));};
          int valid_joints = std::count_if(skeleton.joints.begin(),
                                           skeleton.joints.end(),
                                           valid_lambda);
          if (valid_joints < _minimum_number_of_valid_3D_joints)
          {
            continue;
          }

          skeleton.info = "rtpose with MPI model";
          skeleton.header = _depth_frame_from_sensor.header;
          // TMP
          //////
          if(debug_count++ == 0)
            ROS_WARN_STREAM("rtpose TODO: define confidence, height, distance and occluded policy");
          skeleton.confidence = 100;
          skeleton.height = 0.0;
          skeleton.distance = 0.0;
          skeleton.occluded = false;
          //////
          skeleton_array.skeletons.push_back(skeleton);
          //          updateSkeletonMarkerArray(skeleton_marker_array,
          //                                    skeleton,
          //                                    skeleton_array.skeletons.size() - 1);
        }
      }
      _skeleton_pub.publish(skeleton_array);
      //      _marker_pub.publish(skeleton_marker_array);
    }

    if(_raw_skeleton_image_on_topic)
    {
      _raw_skeleton_image_pub.publish
          (cv_bridge::CvImage(_rgb_frame_from_sensor.header, "bgr8",
                              wrap_frame).toImageMsg());
      _very_raw_image_pub.publish
          (cv_bridge::CvImage(_rgb_frame_from_sensor.header, "bgr8",
                              frame_copy).toImageMsg());
      _raw_skeleton_depth_image_pub.publish
          (cv_bridge::CvImage(_depth_frame_from_sensor.header, "bgr8",
                              _colored_depth_image).toImageMsg());
    }
    if (!_no_display) {
      cv::imshow("video2", _colored_depth_image);
      cv::imshow("video", wrap_frame);
    }

    delete [] frame.data_for_mat;
    delete [] frame.data_for_wrap;
    delete [] frame.data;
    //LOG(ERROR) << msg;
    int key = cv::waitKey(1);
    if (!handleKey(key)) {
      // TODO: sync issues?
      ros::shutdown();
      break;
    }
    VLOG(2) << "Display time " << (get_wall_time()-tic)*1000.0 << " ms.";
  }
  return nullptr;
}

int rtcpm() {
  const auto timer_begin = std::chrono::high_resolution_clock::now();

  // Opening processing deep net threads
  pthread_t gpu_threads_pool[NUM_GPU];
  for(int gpu = 0; gpu < NUM_GPU; gpu++) {
    int *arg = new int[1];
    *arg = gpu+_start_device;
    int rc = pthread_create(&gpu_threads_pool[gpu], NULL, processFrame,
                            (void *) arg);
    if (rc) {
      LOG(ERROR) << "Error:unable to create thread," << rc << "\n";
      return -1;
    }
  }
  LOG(INFO) << "Finish spawning " << NUM_GPU << " threads." << "\n";

  // Setting output resolution
  if (!_no_display) {
    cv::namedWindow("video", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    if (_fullscreen) {
      cv::resizeWindow("video", 1920, 1080);
      cv::setWindowProperty("video", CV_WND_PROP_FULLSCREEN,
                            CV_WINDOW_FULLSCREEN);
      global.uistate.is_fullscreen = true;
    } else {
      cv::resizeWindow("video", DISPLAY_RESOLUTION_WIDTH,
                       DISPLAY_RESOLUTION_HEIGHT);
      cv::setWindowProperty("video", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
      global.uistate.is_fullscreen = false;
    }
#if DEBUG > 0
    cv::namedWindow("video2", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
    if (_fullscreen) {
      cv::resizeWindow("video2", 1920, 1080);
      cv::setWindowProperty("video2", CV_WND_PROP_FULLSCREEN,
                            CV_WINDOW_FULLSCREEN);
      global.uistate.is_fullscreen = true;
    } else {
      cv::resizeWindow("video2", DISPLAY_RESOLUTION_WIDTH,
                       DISPLAY_RESOLUTION_HEIGHT);
      cv::setWindowProperty("video2", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
      global.uistate.is_fullscreen = false;
    }
#endif
  }
  // Openning frames producer (e.g. video, webcam) threads
  usleep(3 * 1e6);
  int thread_pool_size = 1;
  pthread_t threads_pool[thread_pool_size];
  for(int i = 0; i < thread_pool_size; i++) {
    int *arg = new int[i];
    int rc = pthread_create(&threads_pool[i], NULL, getFrames, (void *) arg);
    if (rc) {
      LOG(ERROR) << "Error: unable to create thread," << rc << "\n";
      return -1;
    }
  }
  VLOG(3) << "Finish spawning " << thread_pool_size
          << " threads. now waiting." << "\n";

  // threads handling outputs
  int thread_pool_size_out = NUM_GPU;
  pthread_t threads_pool_out[thread_pool_size_out];
  for(int i = 0; i < thread_pool_size_out; i++) {
    int *arg = new int[i];
    int rc = pthread_create(&threads_pool_out[i], NULL, postProcessFrame,
                            (void *) arg);
    if (rc) {
      LOG(ERROR) << "Error: unable to create thread," << rc << "\n";
      return -1;
    }
  }
  VLOG(3) << "Finish spawning " << thread_pool_size_out
          << " threads. now waiting." << "\n";

  // thread for buffer and ordering frame
  pthread_t threads_order;
  int *arg = new int[1];
  int rc = pthread_create(&threads_order, NULL, buffer_and_order, (void *) arg);
  if (rc) {
    LOG(ERROR) << "Error: unable to create thread," << rc << "\n";
    return -1;
  }
  VLOG(3) << "Finish spawning the thread for ordering. now waiting." << "\n";

  // display
  pthread_t thread_display;
  rc = pthread_create(&thread_display, NULL, displayFrame, (void *) arg);
  if (rc) {
    LOG(ERROR) << "Error: unable to create thread," << rc << "\n";
    return -1;
  }
  VLOG(3) << "Finish spawning the thread for display. now waiting." << "\n";

  // Joining threads
  for (int i = 0; i < thread_pool_size; i++) {
    pthread_join(threads_pool[i], NULL);
  }
  for (int i = 0; i < NUM_GPU; i++) {
    pthread_join(gpu_threads_pool[i], NULL);
  }

  LOG(ERROR) << "rtcpm successfully finished.";

  const auto total_time_sec =
      (double)std::chrono::duration_cast<std::chrono::nanoseconds>
      (std::chrono::high_resolution_clock::now()-timer_begin).count() * 1e-9;
  LOG(ERROR) << "Total time: " << total_time_sec << " seconds.";

  ROS_INFO_STREAM("rtcpm() Finished!");

  return 0;
}

bool handleKey(int c) {
  const std::string key2part = "0123456789qwertyuiopas";
  VLOG(4) << "key: " << (char)c << " code: " << c;
  if (c>=65505) {
    global.uistate.is_shift_down = true;
    c = (char)c;
    c = tolower(c);
    VLOG(4) << "post key: " << (char)c << " code: " << c;
  } else {
    global.uistate.is_shift_down = false;
  }
  VLOG(4) << "shift: " << global.uistate.is_shift_down;
  if (c==27) {
    global.quit_threads = true;
    return false;
  }

  if (c=='g') {
    global.uistate.is_googly_eyes = !global.uistate.is_googly_eyes;
  }

  // Rudimentary seeking in video
  if (c=='l' || c=='k' || c==' ') {
    if (!_video.empty()) {
      int cur_frame = global.uistate.current_frame;
      int frame_delta = 30;
      if (global.uistate.is_shift_down)
        frame_delta = 2;
      if (c=='l') {
        VLOG(4) << "Jump " << frame_delta << " frames to " << cur_frame;
        global.uistate.current_frame+=frame_delta;
        global.uistate.seek_to_frame = 1;
      } else if (c=='k') {
        VLOG(4) << "Rewind " << frame_delta << " frames to " << cur_frame;
        global.uistate.current_frame-=frame_delta;
        global.uistate.seek_to_frame = 1;
      }
    }
    if (c==' ') {
      global.uistate.is_video_paused = !global.uistate.is_video_paused;
    }
  }
  if (c=='f') {
    if (!global.uistate.is_fullscreen) {
      cv::namedWindow("video", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
      cv::resizeWindow("video", 1920, 1080);
      cv::setWindowProperty("video", CV_WND_PROP_FULLSCREEN,
                            CV_WINDOW_FULLSCREEN);
      global.uistate.is_fullscreen = true;
    } else {
      cv::namedWindow("video", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);
      cv::setWindowProperty("video", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
      cv::resizeWindow("video", DISPLAY_RESOLUTION_WIDTH,
                       DISPLAY_RESOLUTION_HEIGHT);
      global.uistate.is_fullscreen = false;
    }
  }
  int target = -1;
  int ind = key2part.find(c);
  if (ind!=std::string::npos) {// && !global.uistate.is_shift_down) {
    target = ind;
  }

  if (target >= 0 && target <= 42) {
    global.part_to_show = target;
    LOG(INFO) << "p2s: " << global.part_to_show;
  }

  if (c=='-' || c=='=') {
    if (c=='-')
      global.nms_threshold -= 0.005;
    if (c=='=')
      global.nms_threshold += 0.005;
    LOG(INFO) << "nms_threshold: " << global.nms_threshold;
  }
  if (c=='_' || c=='+') {
    if (c=='_')
      global.connect_min_subset_score -= 0.005;
    if (c=='+')
      global.connect_min_subset_score += 0.005;
    LOG(INFO) << "connect_min_subset_score: "
              << global.connect_min_subset_score;
  }
  if (c=='[' || c==']') {
    if (c=='[')
      global.connect_inter_threshold -= 0.005;
    if (c==']')
      global.connect_inter_threshold += 0.005;
    LOG(INFO) << "connect_inter_threshold: "
              << global.connect_inter_threshold;
  }
  if (c=='{' || c=='}') {
    if (c=='{')
      global.connect_inter_min_above_threshold -= 1;
    if (c=='}')
      global.connect_inter_min_above_threshold += 1;
    LOG(INFO) << "connect_inter_min_above_threshold: "
              << global.connect_inter_min_above_threshold;
  }
  if (c==';' || c=='\'') {
    if (c==';')
      global.connect_min_subset_cnt -= 1;
    if (c=='\'')
      global.connect_min_subset_cnt += 1;
    LOG(INFO) << "connect_min_subset_cnt: " << global.connect_min_subset_cnt;
  }

  if (c==',' || c=='.') {
    if (c=='.')
      global.part_to_show++;
    if (c==',')
      global.part_to_show--;
    if (global.part_to_show<0) {
      global.part_to_show = 42;
    }
    // if (global.part_to_show>42) {
    //     global.part_to_show = 0;
    // }
    if (global.part_to_show>55) {
      global.part_to_show = 0;
    }
    LOG(INFO) << "p2s: " << global.part_to_show;
  }

  return true;
}

// The global parameters must be assign after the main has started,
// not statically before. Otherwise, they will take the default values,
// not the user-introduced values.
int setGlobalParametersFromFlags() {
  if (DISPLAY_RESOLUTION_WIDTH==-1 && !_video.empty()) {
    cv::VideoCapture cap;
    CHECK(cap.open(_video)) << "Couldn't open video " << _video;
    DISPLAY_RESOLUTION_WIDTH = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    DISPLAY_RESOLUTION_HEIGHT = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    LOG(INFO) << "Setting display resolution from video: "
              << DISPLAY_RESOLUTION_WIDTH << "x" << DISPLAY_RESOLUTION_HEIGHT;
  } else if (DISPLAY_RESOLUTION_WIDTH==-1 && !_image_dir.empty()) {
    cv::Mat image_uchar_orig = cv::imread(global.image_list[0].c_str(),
        CV_LOAD_IMAGE_COLOR);
    DISPLAY_RESOLUTION_WIDTH = image_uchar_orig.cols;
    DISPLAY_RESOLUTION_HEIGHT = image_uchar_orig.rows;
    LOG(INFO) << "Setting display resolution from first image: "
              << DISPLAY_RESOLUTION_WIDTH << "x" << DISPLAY_RESOLUTION_HEIGHT;
  } else if (DISPLAY_RESOLUTION_WIDTH==-1) {
    LOG(ERROR) << "Invalid resolution without video/images: "
               << DISPLAY_RESOLUTION_WIDTH << "x" << DISPLAY_RESOLUTION_HEIGHT;
    exit(1);
  } else {
    LOG(INFO) << "Display resolution: " << DISPLAY_RESOLUTION_WIDTH
              << "x" << DISPLAY_RESOLUTION_HEIGHT;
  }
  //  nRead = sscanf(_camera_resolution.c_str(), "%dx%d",
  //                 &CAMERA_FRAME_WIDTH, &CAMERA_FRAME_HEIGHT);
  //  CHECK_EQ(nRead,2) << "Error, camera resolution format ("
  //                    <<  _camera_resolution
  //                     << ") invalid, should be e.g., 1280x720";
  int nRead = sscanf(_net_resolution.c_str(), "%dx%d",
                     &NET_RESOLUTION_WIDTH, &NET_RESOLUTION_HEIGHT);
  CHECK_EQ(nRead,2) << "Error, net resolution format ("
                    <<  _net_resolution
                     << ") invalid, should be e.g., 656x368 (multiples of 16)";
  LOG(INFO) << "Net resolution: " << NET_RESOLUTION_WIDTH
            << "x" << NET_RESOLUTION_HEIGHT;

  if (!_write_frames.empty()) {
    // Create folder if it does not exist
    boost::filesystem::path dir(_write_frames);
    if (!boost::filesystem::is_directory(dir) &&
        !boost::filesystem::create_directory(dir)) {
      LOG(ERROR) << "Could not write to or create directory " << dir;
      return 1;
    }
  }

  if (!_write_json.empty()) {
    // Create folder if it does not exist
    boost::filesystem::path dir(_write_json);
    if (!boost::filesystem::is_directory(dir) &&
        !boost::filesystem::create_directory(dir)) {
      LOG(ERROR) << "Could not write to or create directory " << dir;
      return 1;
    }
  }

  BATCH_SIZE = {_num_scales};
  SCALE_GAP = {_scale_gap};
  START_SCALE = {_start_scale};
  NUM_GPU = {_num_gpu};
  // Global struct/classes
  global.part_to_show = _part_to_show;
  net_copies = std::vector<NetCopy>(NUM_GPU);
  // Set nets
  PERSON_DETECTOR_CAFFEMODEL = _caffe_model;
  PERSON_DETECTOR_PROTO = _caffe_proto;

  return 0;
}

int readImageDirIfFlagEnabled()
{
  // Open & read image dir if present
  if (!_image_dir.empty()) {
    std::string folderName = _image_dir;
    if ( !boost::filesystem::exists( folderName ) ) {
      LOG(ERROR) << "Folder " << folderName << " does not exist.";
      return -1;
    }
    boost::filesystem::directory_iterator end_itr;  // default construction
    // yields past-the-end
    for ( boost::filesystem::directory_iterator itr( folderName );
          itr != end_itr; ++itr ) {
      if ( boost::filesystem::is_directory(itr->status()) ) {
        // Skip directories
      } else if (itr->path().extension()==".jpg" ||
                 itr->path().extension()==".png" ||
                 itr->path().extension()==".bmp") {
        //  std::string filename = itr->path().string();
        global.image_list.push_back( itr->path().string() );
      }
    }
    std::sort(global.image_list.begin(), global.image_list.end());
    CHECK_GE(global.image_list.size(),0);
  }

  return 0;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "body_pose_estimator");
  ros::NodeHandle pnh("~"), nh;

  // Parameters
  pnh.param<bool>("fullscreen", _fullscreen, false);
  pnh.param<int>("part_to_show", _part_to_show, 0);
  pnh.param<std::string>("write_frames", _write_frames, "");
  pnh.param<bool>("no_frame_drops", _no_frame_drops, false);
  pnh.param<std::string>("write_json", _write_json, "");
  pnh.param<int>("camera_index", _camera_index, 0);
  pnh.param<std::string>("video", _video, "");
  pnh.param<std::string>("image_dir", _image_dir, "");
  pnh.param<int>("start_frame", _start_frame, 0);
  pnh.param<std::string>("caffe_model", _caffe_model,
                         ros::package::getPath("body_pose_estimation") +
                         "/model/coco/pose_iter_440000.caffemodel");
  pnh.param<std::string>("caffe_proto", _caffe_proto,
                         ros::package::getPath("body_pose_estimation") +
                         "/model/coco/pose_deploy_linevec.prototxt");
  pnh.param<std::string>("resolution", _resolution, "640x480");
  pnh.param<std::string>("net_resolution", _net_resolution, "320x160");
  pnh.param<std::string>("camera_resolution", _camera_resolution, "640x480");
  pnh.param<int>("start_device", _start_device, 0);
  pnh.param<int>("num_gpu", _num_gpu, 1);
  pnh.param<double>("start_scale", _start_scale, 1);
  pnh.param<double>("scale_gap", _scale_gap, 0.3);
  pnh.param<int>("num_scales", _num_scales, 1);
  pnh.param<bool>("no_display", _no_display, false);
  pnh.param<std::string>("skeleton_topic_to_publish",
                         _skeleton_topic_to_publish, "");
  pnh.param<std::string>("camera_info_topic", _camera_info_topic, "");
  pnh.param<std::string>("rgb_topic", _rgb_topic, "");
  pnh.param<std::string>("depth_topic", _depth_topic, "");
  pnh.param<std::string>("marker_topic_to_publish", _marker_topic_to_publish,
                         "");
  pnh.param<int>("median_search_x_component", _median_search_x_component, 7);
  pnh.param<int>("median_search_y_component", _median_search_y_component, 7);
  pnh.param<bool>("head_joint_enhancement", _head_joint_enhancement, true);
  pnh.param<double>("head_correction_param", _head_correction_param, 0.3);
  pnh.param<int>("max_size_input_queue", _max_size_input_queue, 1);
  pnh.param<bool>("skeleton_solid_color", _skeleton_solid_color, false);
  pnh.param<double>("skeleton_color_r", _skeleton_color.x, 1.0);
  pnh.param<double>("skeleton_color_g", _skeleton_color.y, 0.0);
  pnh.param<double>("skeleton_color_b", _skeleton_color.z, 0.0);
  pnh.param<double>("noise_threshold", _noise_threshold, 1.0);
  pnh.param<int>("minimum_number_of_valid_3D_joints",
                 _minimum_number_of_valid_3D_joints, 5);
  pnh.param<std::string>("raw_skeleton_image_topic_to_publish",
                         _raw_skeleton_image_topic_to_publish,
                         "");
  pnh.param<bool>("raw_skeleton_image_on_topic",
                  _raw_skeleton_image_on_topic,
                  true);

  dynamic_reconfigure::Server<rtpose_wrapper::JointsConfig> server;
  dynamic_reconfigure::Server<rtpose_wrapper::JointsConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  message_filters::Subscriber<sensor_msgs::CameraInfo> sub1_(nh,
                                                             _camera_info_topic,
                                                             1);
  message_filters::Subscriber<sensor_msgs::Image> sub2_(nh,
                                                        _rgb_topic,
                                                        1);
  message_filters::Subscriber<sensor_msgs::Image> sub3_(nh,
                                                        _depth_topic,
                                                        1);
  typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::CameraInfo,
      sensor_msgs::Image,
      sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10),
                                                    sub1_,
                                                    sub2_,
                                                    sub3_);

  //Publisher
  if(!_skeleton_topic_to_publish.empty())
  {
    _skeleton_pub = nh.advertise<rtpose_wrapper::SkeletonArrayMsg>
        (_skeleton_topic_to_publish, 1);
    //    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>
    //        ("/detector/skeleton_array_markers", 1);
  }
  // skeleton image publisher
  if(_raw_skeleton_image_on_topic
     and not _raw_skeleton_image_topic_to_publish.empty())
  {
    _raw_skeleton_image_pub = nh.advertise<sensor_msgs::Image>
        (_raw_skeleton_image_topic_to_publish, 1);
    _very_raw_image_pub = nh.advertise<sensor_msgs::Image>
        ("/detector/image_raw", 1);
  }
  _raw_skeleton_depth_image_pub = nh.advertise<sensor_msgs::Image>
      ("/detector/skeletons_depth_image", 1);
  //Subscriber
  if(!_rgb_topic.empty()
     && !_camera_info_topic.empty()
     && !_depth_topic.empty())
  {
    ros::Rate rate(20);
    while(_skeleton_pub.getNumSubscribers() == 0
          or sub1_.getSubscriber().getNumPublishers() == 0
          or sub2_.getSubscriber().getNumPublishers() == 0
          or sub3_.getSubscriber().getNumPublishers() == 0)
    {
      ROS_INFO_STREAM("No subscriber to skeletons and/or "
                      "no publisher for input topics, waiting....");
      ros::spinOnce();
      rate.sleep();
      if(!ros::ok) ros::shutdown();
    }
    // TODO: while there are no publishers wait...
    sync_.registerCallback(boost::bind(syncronizedCallback, _1, _2, _3));
    ros::spinOnce();
  }
  else
  {
    ROS_INFO_STREAM("No topics specified... I am assuming you want "
                    "to work with webcam or video or images");
  }

  // I need an async spinner which uses another thread for calling my callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //  while(_not_ready)
  //  {
  //    ros::spinOnce();
  //    ros::Rate(30).sleep();
  //    if(!ros::ok()) ros::shutdown();
  //  }

  // Initializing google logging (Caffe uses it as its logging module)
  google::InitGoogleLogging("rtcpm");

  // Parsing command line flags
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Applying user defined configuration and/or default parameter
  // values to global parameters
  auto return_value = setGlobalParametersFromFlags();
  if (return_value != 0)
    return return_value;

  // Configure frames source
  return_value = readImageDirIfFlagEnabled();
  if (return_value != 0)
    return return_value;

  ROS_INFO_STREAM("I am beginning to stream skeletons...");

  // Running rtcpm
  return_value = rtcpm();
  if (return_value != 0)
    return return_value;

  ros::shutdown();

  return return_value;
}

#include <mutex>
#include <memory>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <message_filters/time_synchronizer.h>

#include <opt_msgs/IDArray.h>
#include <opt_msgs/NameArray.h>
#include <opt_msgs/TrackArray.h>
#include <opt_msgs/DetectionArray.h>
#include <opt_msgs/Association.h>
#include <opt_msgs/FeatureVectorArray.h>

#include <dynamic_reconfigure/server.h>
#include <recognition/FaceRecognitionConfig.h>
#include <recognition/OPTSetPredefinedFaces.h>
#include <recognition/OPTSaveRegisteredFaces.h>
#include <recognition/OPTLoadRegisteredFaces.h>

#include <open_ptrack/recognition/face_recognizer.hpp>
#include <open_ptrack/recognition/nn/face_recognizer_nn.hpp>
#include <open_ptrack/recognition/bayes/face_recognizer_bayes.hpp>

/**
 * @brief The FaceRecognitionNode
 */
class FaceRecognitionNode {
public:
  /**
   * @brief constructor
   * @param nh node handler
   */
  FaceRecognitionNode(ros::NodeHandle& nh)
    : set_predefined_faces_service(nh.advertiseService("/face_recognition/set_predefined_faces", &FaceRecognitionNode::set_predefined_faces, this)),
      save_registered_faces_service(nh.advertiseService("/face_recognition/save_registered_faces", &FaceRecognitionNode::save_registered_faces, this)),
      load_registered_faces_service(nh.advertiseService("/face_recognition/load_registered_faces", &FaceRecognitionNode::load_registered_faces, this)),
      track_pub(nh.advertise<opt_msgs::TrackArray>("/face_recognition/people_tracks", 10)),
      names_pub(nh.advertise<opt_msgs::NameArray>("/face_recognition/people_names", 10)),
//      id_pub(nh.advertise<opt_msgs::TrackArray>()),
      names_pub_timer(nh.createTimer(ros::Duration(0.034), &FaceRecognitionNode::publish_names, this)),
      track_sub(nh.subscribe("/tracker/tracks_smoothed", 10, &FaceRecognitionNode::track_callback, this)),
      association_sub(nh, "/tracker/association_result", 10),
      detections_sub(nh, "/face_detector/detections", 10),
      feature_vector_sub(nh, "/face_feature_extractor/features", 10),
      sync(association_sub, detections_sub, feature_vector_sub, 1000),
      seq(ros::Duration(0.1), ros::Duration(0.01), 24)
  {
//    recognizer.reset(new FaceRecognizerNN(0.8, 5));
    recognizer.reset(new FaceRecognizerBayes());
    cfg_server.setCallback(boost::bind(&FaceRecognitionNode::cfg_callback, this, _1, _2));

    features_buffer.reserve(32);
    sync.registerCallback(boost::bind(&FaceRecognitionNode::face_callback, this, _1, _2, _3));
    seq.registerCallback(boost::bind(&FaceRecognitionNode::face_sequenced_callback, this, _1));

    names_seq_num = 0;
  }

private:
  /**
   * @brief callback for dynamic reconfigure
   * @param config  configure parameters
   * @param level   configure level
   */
  void cfg_callback(recognition::FaceRecognitionConfig& config, uint32_t level) {
    std::cout << "--- cfg_callback ---" << std::endl;
    recognizer->cfgCallback(config, level);
  }

  /**
   * @brief receives a TrackArray and outputs a TrackArray which IDs are replaced with face recognition IDs
   * @param track_msg   the input TrackArray
   */
  void track_callback(const opt_msgs::TrackArrayConstPtr& track_msg) {
    opt_msgs::TrackArrayPtr recognized_msg(new opt_msgs::TrackArray());
    *recognized_msg = *track_msg;

    std::unique_lock<std::mutex> lock(recognizer_mutex);
    for(auto& track : recognized_msg->tracks) {
      recognizer->collectGarbage(track_msg);
      track.stable_id = recognizer->convertID(track.id);
    }
    lock.unlock();

    track_pub.publish(recognized_msg);
  }

  /**
   * @brief publishes the set of predefined faces associated with the registered faces
   *        this method is called by a timer function
   * @param e  time callback argument
   */
  void publish_names(const ros::TimerEvent& e) {
    opt_msgs::NameArrayPtr names_msg(new opt_msgs::NameArray());
    names_msg->header.seq = names_seq_num++;
    names_msg->header.stamp = ros::Time::now();

    std::unique_lock<std::mutex> lock(recognizer_mutex);
    const auto& predefined_faces = recognizer->getAssociatedPredefinedFaces();
    names_msg->ids.reserve(predefined_faces.size());
    names_msg->names.reserve(predefined_faces.size());

    for(const auto& face : predefined_faces) {
      names_msg->ids.push_back(face.first);
      names_msg->names.push_back(face.second->getName());
    }
    lock.unlock();

    names_pub.publish(names_msg);
  }

  /**
   * @brief receives an AssociationResult and a FeatureVectorArray
   *        the association data is added to the FeatureVectorArray and sent to the sequencer so that the features sent from several PCs are ordered by those timestamps
   * @param association_msg     the data association result by the tracker
   * @param feature_vector_msg  the face feature vectors
   */
  void face_callback(const opt_msgs::AssociationConstPtr& association_msg, const opt_msgs::DetectionArrayConstPtr& detections_msg, const opt_msgs::FeatureVectorArrayConstPtr& feature_vector_msg) {
    if(association_msg->track_ids.size() != feature_vector_msg->vectors.size() || association_msg->track_ids.size() != detections_msg->detections.size()){
      std::cerr << "warning : the numbers of the trackers and the feature vectors must be same!!" << std::endl;
      std::cerr << "        : skip this frame" << std::endl;
      return;
    }

    if(association_msg->header.frame_id != feature_vector_msg->header.frame_id || association_msg->header.frame_id != detections_msg->header.frame_id) {
      std::cerr << "warning : the data association msg and the feature vector msg from differenct PCs are conflicted!!" << std::endl;
      std::cerr << "        : skip this frame" << std::endl;
    }

    opt_msgs::FeatureVectorArrayPtr features_with_id_msg( new opt_msgs::FeatureVectorArray() );
    features_with_id_msg->header = feature_vector_msg->header;
    features_with_id_msg->ids.reserve(feature_vector_msg->vectors.size());
    features_with_id_msg->vectors.reserve(feature_vector_msg->vectors.size());

int count = feature_vector_msg->vectors.size(); //jb


    for(int i=0; i<feature_vector_msg->vectors.size(); i++) {
      if(feature_vector_msg->vectors[i].data.empty()) {
	count--;
        continue;
      }

      const auto& detection = detections_msg->detections[i];
      Eigen::Vector3f pos(detection.centroid.x, detection.centroid.y, detection.centroid.z);
      if (pos.norm() > 7.5) {
        std::cout << "TOO FAR!!" << std::endl;
	count--;
        continue;
      }
      features_with_id_msg->ids.push_back(association_msg->track_ids[i]);
      features_with_id_msg->vectors.push_back(feature_vector_msg->vectors[i]);
    }
if (count > 0) { //jb 
    face_sequenced_callback(features_with_id_msg);
}
//    seq.add(features_with_id_msg);
  }

  /**
   * @brief receives a sequenced FeatureVectorArray and updates the face recognizer
   * @param feature_vector_msg  the sequenced FeatureVectorArray
   */
  void face_sequenced_callback(const opt_msgs::FeatureVectorArrayConstPtr& feature_vector_msg) {
    for(int i=0; i<feature_vector_msg->ids.size(); i++) {
      const auto& vector = feature_vector_msg->vectors[i];
      Eigen::Map<Eigen::MatrixXf> map(const_cast<float*>(vector.data.data()), vector.layout.dim[0].size, 1);
      std::shared_ptr<Eigen::VectorXf> v( new Eigen::VectorXf(map) );
      std::cout << "test facesequencedcallback" << std::endl;
      features_buffer.push_back( std::tuple<ros::Time, int, std::shared_ptr<Eigen::VectorXf>>( feature_vector_msg->header.stamp, feature_vector_msg->ids[i], v ) );
    }
    flush_features_buffer();
    //
/*    ros::Duration duration = (std::get<0>(features_buffer.back()) - std::get<0>(features_buffer.front()));
    if( duration.toSec() >= 0.1 ) {
      flush_features_buffer();
    }
    */
  }

  /**
   * @brief updates the face recognizer with the data in the buffer and then clears the buffer
   */
  void flush_features_buffer() {
    std::cout << "testupdate" << std::endl;

    std::unordered_map<int, std::vector<std::shared_ptr<Eigen::VectorXf>>> fmap;
    for(const auto& features : features_buffer) {
      fmap[std::get<1>(features)].push_back(std::get<2>(features));
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex);
    recognizer->update(fmap);
    features_buffer.clear();

    recognizer->updatePredefined();
  }

  /**
   * @brief set_predefined_faces
   * @param req
   * @param res
   * @return
   */
  bool set_predefined_faces(recognition::OPTSetPredefinedFaces::Request& req, recognition::OPTSetPredefinedFaces::Response& res) {
    std::cout << "received faces here" << std::endl;

    std::vector<std::tuple<std::string, cv::Mat, Eigen::VectorXf>> predefined_faces(req.faces.size());
   for(int i=0; i<req.faces.size(); i++) {
      std::get<0>(predefined_faces[i]) = req.names[i];
      std::get<1>(predefined_faces[i]) = cv_bridge::toCvCopy(req.faces[i])->image;
      std::get<2>(predefined_faces[i]) = Eigen::Map<Eigen::VectorXf>(&req.features[i].data[0], req.features[i].layout.dim[0].size);
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex);
    recognizer->setPredefinedFaces(predefined_faces);
    std::cout << "true" << std::endl;
    return true;
  }

  bool save_registered_faces(recognition::OPTSaveRegisteredFaces::Request& req, recognition::OPTSaveRegisteredFaces::Response& res) {
    std::lock_guard<std::mutex> lock(recognizer_mutex);
    std::cout << "--- save_regitered_faces ---" << std::endl;

    auto faces = recognizer->getRegisteredFaces();

    std::ofstream ofs(req.path);
    if(!ofs) {
      std::cerr << "error : failed to open the file!!" << std::endl;
      std::cerr << "      : " << req.path << std::endl;
      res.status = res.STATUS_ERROR;
      return true;
    }

    ofs << "num_faces " << faces.size() << std::endl;
    ofs << "face_id num_features" << std::endl;
    for(const auto& face : faces) {
      ofs << face->getFaceId() << " " << face->getFaces().size() << std::endl;
    }
    ofs << "features" << std::endl;
    for(const auto& face : faces) {
      for(const auto& feature : face->getFaces()) {
        ofs << feature.transpose() << std::endl;
      }
    }

    return true;
  }

  bool load_registered_faces(recognition::OPTLoadRegisteredFaces::Request& req, recognition::OPTLoadRegisteredFaces::Response& res) {
    std::cout << "--- load_regitered_faces ---" << std::endl;
    std::ifstream ifs(req.path);
    if(!ifs) {
      std::cerr << "error : failed to open the file!!" << std::endl;
      std::cerr << "      : " << req.path << std::endl;
      res.status = res.STATUS_ERROR;
      return true;
    }

    std::string token;
    int num_faces;

    ifs >> token >> num_faces;
    ifs >> token >> token;    // ignore "face_id num_features"
    std::cout << token << std::endl;

    if(num_faces == 0) {
      std::cerr << "empty!!" << std::endl;
      res.status = res.STATUS_ERROR;
      return true;
    }

    std::vector<RegisteredFace::Ptr> faces(num_faces);
    for(auto& face : faces) {
      int face_id, num_features;
      ifs >> face_id >> num_features;
      std::cout << face_id << " " << num_features << std::endl;

      face.reset(new RegisteredFace(face_id));
      face->getFaces().resize(num_features);
    }

    ifs >> token;   // ignore "features"
    for(auto& face : faces) {
      for(auto& feature : face->getFaces()) {
        feature.resize(128);
        for(int i=0; i<feature.size(); i++) {
          ifs >> feature[i];
        }
      }
    }

    std::lock_guard<std::mutex> lock(recognizer_mutex);
    recognizer->registerFaces(faces);
    return true;
  }

private:
  // face recognizer
  std::mutex recognizer_mutex;
  std::unique_ptr<FaceRecognizer> recognizer;

  // ROS
  ros::ServiceServer set_predefined_faces_service;
  ros::ServiceServer save_registered_faces_service;
  ros::ServiceServer load_registered_faces_service;
  dynamic_reconfigure::Server<recognition::FaceRecognitionConfig> cfg_server;

  ros::Publisher track_pub;
  ros::Publisher names_pub;
  ros::Timer names_pub_timer;

  ros::Subscriber track_sub;
  ros::Subscriber alive_ids_sub;

  message_filters::Subscriber<opt_msgs::Association> association_sub;
  message_filters::Subscriber<opt_msgs::DetectionArray> detections_sub;
  message_filters::Subscriber<opt_msgs::FeatureVectorArray> feature_vector_sub;
  message_filters::TimeSynchronizer<opt_msgs::Association, opt_msgs::DetectionArray, opt_msgs::FeatureVectorArray> sync;

  message_filters::TimeSequencer<opt_msgs::FeatureVectorArray> seq;

  std::vector<std::tuple<ros::Time, int, std::shared_ptr<Eigen::VectorXf>>> features_buffer;

  int names_seq_num;
};


int main(int argc, char** argv) {
  std::cout << "--- face_recognition_node ---" << std::endl;
  ros::init(argc, argv, "face_recognition_node");
  ros::NodeHandle nh;

  FaceRecognitionNode node(nh);
  ros::spin();
  return 0;
}

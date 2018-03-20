#ifndef FACE_RECOGNIZER_NN_HPP
#define FACE_RECOGNIZER_NN_HPP

#include <memory>
#include <iostream>
#include <unordered_map>
#include <opt_msgs/IDArray.h>
#include <opt_msgs/TrackArray.h>

#include "open_ptrack/recognition/face_recognizer.hpp"
#include "open_ptrack/recognition/registered_face.hpp"
#include "open_ptrack/recognition/nn/tracker_status_nn.hpp"

/**
 * @brief face recognition based on k-nearest neighbor matching
 */
class FaceRecognizerNN : public FaceRecognizer {
public:

  /**
   * @brief constructor
   * @param fp_threshold    the threshold for false positive detection
   * @param num_neighbors   the number of neighbor points used for kNN
   */
  FaceRecognizerNN(double fp_threshold = 0.8, int num_neighbors = 5) {
    min_voting_faces = 5;
    min_support_faces = 3;

    this->fp_threshold = 2.75;     //
    this->num_neighbors = 5;   //

    face_id_source = 0;
  }

  ~FaceRecognizerNN() {}

  /**
   * @brief updates the face recognizer
   * @param a set of tracker ids and feature vectors
   */
  void update(const std::unordered_map<int, std::vector<std::shared_ptr<Eigen::VectorXf>>>& fmap) override {
    for(const auto& f : fmap) {
      update(f.first, f.second);
    }
  }

  /**
   * @brief updatePredefined
   */
  void updatePredefined() override {
    int predefined_recognition_min_images = 10;
    int predefined_recognition_num_test_images = 5;
    double predefined_recognition_fp_thresh = 0.65;

    for(const auto& face : associated_faces) {
      if(face.second->size() < predefined_recognition_min_images) {
        continue;
      }

      if(associated_predefined_faces.find(face.second->getFaceId()) != associated_predefined_faces.end()) {
        continue;
      }

      RegisteredFace::Ptr closest_predefined;
      double closest_dist = std::numeric_limits<double>::max();

      for(const auto& predefined : unassociated_predefined_faces) {
        double dist = predefined->calcDistance(*face.second, predefined_recognition_num_test_images);
        std::cout << face.second->getFaceId() << " - " << predefined->getName() << " : " << dist << std::endl;
        if ( dist < closest_dist ) {
          closest_dist = dist;
          closest_predefined = predefined;
        }
      }

      if (closest_dist > predefined_recognition_fp_thresh) {
        continue;
      }

      associated_predefined_faces[face.second->getFaceId()] = closest_predefined;
      unassociated_predefined_faces.erase(std::find(unassociated_predefined_faces.begin(), unassociated_predefined_faces.end(), closest_predefined));
    }
  }

  /**
   * @brief converts a tracker ID to a face recognition ID
   * @param tracker_id  the tracker ID
   * @return the face recognition ID
   */
  int convertID(int tracker_id) const override {
    auto loc = associated_faces.find(tracker_id);
    if(loc != associated_faces.end()) {
      return loc->second->getFaceId();
    }

    return tracker_id + 10000;
  }

  /**
   * @brief disposes unnecessary data
   * @param trackers trackers
   */
  void collectGarbage(const opt_msgs::TrackArrayConstPtr& trackers) override {
    for(auto tracker = tracker_status_map.begin(); tracker != tracker_status_map.end(); tracker++ ){
      auto found = std::find_if(trackers->tracks.begin(), trackers->tracks.end(),
        [&](const opt_msgs::Track& t) {return t.id == tracker->first;}
      );

      if(found == trackers->tracks.end()) {
        tracker_status_map.erase(tracker);
        return collectGarbage(trackers);
      }
    }

    for(auto face = associated_faces.begin(); face != associated_faces.end(); face++) {
      auto found = std::find_if(trackers->tracks.begin(), trackers->tracks.end(), [&](const opt_msgs::Track& t) {return t.id == face->first;});

      if(found == trackers->tracks.end()) {
        std::cout << "ERASE!!" << std::endl;
        face->second->setTrackerId(-1);
        unassociated_faces.push_back(face->second);
        associated_faces.erase(face);
        return collectGarbage(trackers);
      }
    }
  }

  void setPredefinedFaces(const std::vector<std::tuple<std::string, cv::Mat, Eigen::VectorXf>>& predefined_faces) override {
    associated_predefined_faces.clear();
    unassociated_predefined_faces.clear();

    for(const auto& face : predefined_faces) {
      unassociated_predefined_faces.emplace_back();
      unassociated_predefined_faces.back().reset(new RegisteredFace(std::get<0>(face)));
      unassociated_predefined_faces.back()->addFace(std::get<2>(face));
    }
  }

  /**
   * @brief the set of predefined faces associated with the registered faces
   * @return  the set of predefined faces
   */
  const std::unordered_map<int, RegisteredFace::Ptr>& getAssociatedPredefinedFaces() const override {
    return associated_predefined_faces;
  }

  /**
   * @brief registers faces to the face recognizer
   * @param faces  faces to be registered
   */
  void registerFaces(const std::vector<RegisteredFace::Ptr>& faces) override {
    for(const auto& face : faces) {
      // check if the face_id exists in associated_faces
      auto found_associated = std::find_if(associated_faces.begin(), associated_faces.end(),
        [=](const std::pair<int, RegisteredFace::Ptr>& p) {
          return p.second->getFaceId() == face->getFaceId();
        }
      );
      if (found_associated != associated_faces.end()){
        std::cerr << "warning : face_id (" << face->getFaceId() << ") is already registered" << std::endl;
        found_associated->second->addFaces(face);
        continue;
      }

      // check if the face_id exists in unassociated_faces
      auto found_unassocitead = std::find_if(unassociated_faces.begin(), unassociated_faces.end(),
        [=](const RegisteredFace::Ptr& p) {
          return p->getFaceId() == face->getFaceId();
        }
      );
      if (found_unassocitead != unassociated_faces.end()) {
        std::cerr << "warning : face_id(" << face->getFaceId() << ") is already registered" << std::endl;
        (*found_unassocitead)->addFaces(face);
        continue;
      }

      // the face_id has not been registered
      std::cout << "register face_id(" << face->getFaceId() << ")" << std::endl;
      unassociated_faces.push_back(face);
    }
  }

  /**
   * @brief the set of registered faces
   * @return  the set of registered faces
   */
  std::vector<RegisteredFace::Ptr> getRegisteredFaces() const override {
    std::vector<RegisteredFace::Ptr> registered_faces;
    registered_faces.reserve(unassociated_faces.size() + associated_faces.size());
    std::transform(associated_faces.begin(), associated_faces.end(), std::back_inserter(registered_faces), [=](const std::pair<int, RegisteredFace::Ptr>& p) {return p.second;});
    std::copy(unassociated_faces.begin(), unassociated_faces.end(), std::back_inserter(registered_faces));
    return registered_faces;
  }

  /**
   * @brief callback for dynamic reconfigure
   * @param config  configuration parameters
   * @param level   configuration level
   */
  void cfgCallback(recognition::FaceRecognitionConfig& config, uint32_t level) override {
    /*
    min_voting_faces = config.min_voting_faces;
    min_support_faces = config.min_support_faces;
    fp_threshold = config.fp_threshold;
    num_neighbors = config.num_neighbors;
    */
  }
private:
  /**
   * @brief updates the face recognizer
   * @param tracker_id
   * @param features
   */
  void update(int tracker_id, const std::vector<std::shared_ptr<Eigen::VectorXf>>& features) {
    auto associated_face = associated_faces.find(tracker_id);
    if(associated_face != associated_faces.end()) {
      for(const auto& feature: features) {
        associated_face->second->addFace(*feature);
      }
      return;
    }

    auto& status = tracker_status_map[tracker_id];
    if(status == nullptr) {
      status.reset(new TrackerStatusNN(tracker_id));
    }

    for(const auto& feature : features) {
      auto closest = findClosest(*feature);
      std::cout << closest.first << std::endl;
      if(closest.first > fp_threshold) {
        status->addResult(feature, nullptr);
      } else {
        status->addResult(feature, closest.second);
      }
    }

    assign(status);
  }

  /**
   * @brief find the face which is closest to the given feature vector
   * @param feature  face feature vector
   * @return the distance between #feature and the closest face, and the closest face
   */
  std::pair<double, RegisteredFace::Ptr> findClosest(const Eigen::VectorXf& feature) const {
    double closest_dist = std::numeric_limits<double>::max();
    RegisteredFace::Ptr closest_face;
    for(const auto& face : unassociated_faces) {
      double dist = face->calcDistance(feature);
      if(dist < closest_dist) {
        closest_dist = dist;
        closest_face = face;
      }
    }

    return std::make_pair(closest_dist, closest_face);
  }

  /**
   * @brief assign a face ID to the tracker if valid
   * @param status the recognition status of the tracker
   */
  void assign(TrackerStatusNN::Ptr status) {
    // pair of <face_id and num_votes>
    std::vector<std::pair<int, int>> voting;
    voting.reserve(status->getResults().size());

    int num_faces = 0;
    for(const auto& result : status->getResults()) {
      // the face is already assigned to another tracker
      if(result && result->getTrackerId() >= 0) {
        continue;
      }

      // push
      int face_id = result ? result->getFaceId() : -1;
      auto found = std::find_if(voting.begin(), voting.end(), [&](const std::pair<int, int>& v) { return v.first == face_id; });
      if(found != voting.end()) {
        found->second ++;
      } else {
        voting.push_back(std::make_pair(face_id, 1));
      }
      num_faces ++;
    }

    if(num_faces < min_voting_faces) {
      return;
    }

    auto max_elem = std::max_element(voting.begin(), voting.end(), [=](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {return lhs.second < rhs.second;});
    if(max_elem->second < min_support_faces) {
      return;
    }

    if(max_elem->first < 0) {
      std::cout << "new person!!" << std::endl;
      auto face = RegisteredFace::Ptr(new RegisteredFace(face_id_source++, status->getTrackerId()));
      associated_faces[status->getTrackerId()] = face;
    } else {
      std::cout << "known person!!" << std::endl;
      auto found = std::find_if(unassociated_faces.begin(), unassociated_faces.end(),  [&](const RegisteredFace::Ptr& p) {return p->getFaceId() == max_elem->first;});
      if (found == unassociated_faces.end()) {
        std::cerr << "error : illegal condition!!" << std::endl;
      }

      associated_faces[status->getTrackerId()] = *found;
      unassociated_faces.erase(found);
    }

    auto& face = associated_faces[status->getTrackerId()];
    for(const auto& feature : status->getFeatures()) {
      face->addFace(*feature);
    }

    auto found = std::find_if(tracker_status_map.begin(), tracker_status_map.end(), [&](const std::pair<int, TrackerStatusNN::Ptr>& p) { return p.second == status; } );
    tracker_status_map.erase(found);
  }

private:
  int face_id_source;                                                 // face id generation source
  std::unordered_map<int, TrackerStatusNN::Ptr> tracker_status_map;   //

  std::vector<RegisteredFace::Ptr> unassociated_faces;
  std::unordered_map<int, RegisteredFace::Ptr> associated_faces;

  std::vector<RegisteredFace::Ptr> unassociated_predefined_faces;
  std::unordered_map<int, RegisteredFace::Ptr> associated_predefined_faces;

  int min_voting_faces;
  int min_support_faces;

  int num_neighbors;                      // the number of neighbor points used for classification
  double fp_threshold;                    // the threshold for false positive detection
};

#endif // FACE_RECOGNIZER_HPP

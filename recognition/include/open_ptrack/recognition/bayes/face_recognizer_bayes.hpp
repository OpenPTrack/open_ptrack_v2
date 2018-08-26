#ifndef FACE_RECOGNIZER_BAYES_HPP
#define FACE_RECOGNIZER_BAYES_HPP

#include <memory>
#include <iostream>
#include <unordered_map>
#include <opt_msgs/IDArray.h>
#include <opt_msgs/TrackArray.h>

#include "open_ptrack/math/skew_normal.hpp"
#include "open_ptrack/recognition/face_recognizer.hpp"
#include "open_ptrack/recognition/bayes/registered_face_bayes.hpp"
#include "open_ptrack/recognition/bayes/tracker_status_bayes.hpp"

class FaceRecognizerBayes : public FaceRecognizer {
public:
  FaceRecognizerBayes() {
    face_id_source = 0;
    pos_pdf.reset(new SkewNormalDistribution(2.09329546, 0.51549199, 0.28795632));
    neg_pdf.reset(new SkewNormalDistribution(-3.13951689, 1.54745462, 0.28095953));

    count_thresh = 3;
    posterior_thresh = 0.95;
    neg_pdf_scale = 1.0;
  }
  ~FaceRecognizerBayes() {}

  /**
   * @brief updates the face recognizer
   * @param tracker_id  the id given by the tracker
   * @param features    the face feature vectors
   * @todo it's too long, need to split this function
   */
  void update(const std::unordered_map<int, std::vector<std::shared_ptr<Eigen::VectorXf>>>& fmap) override {
    for(const auto& fset : fmap) {
      int tracker_id = fset.first;

      // check if the tracker is associated with a registered face
      auto found_face = associated_faces.find(tracker_id);
      if(found_face != associated_faces.end()) {
        for(const auto& feature : fset.second) {
          found_face->second->addFace(*feature);
        }
        continue;
      }


      // add the observed faces to the tracker
      auto& tracker_status = tracker_status_map[tracker_id];
      if(tracker_status == nullptr) {
        tracker_status.reset(new TrackerStatusBayes(tracker_id));
      }
      for(const auto& feature : fset.second) {
        tracker_status->addFace(feature);
      }
    }


    // sort the unassociated faces by id
    std::sort(unassociated_faces.begin(), unassociated_faces.end(),
      [=](const RegisteredFace::Ptr& lhs, const RegisteredFace::Ptr& rhs) {
        return lhs->getFaceId() < rhs->getFaceId();
      }
    );


    Eigen::VectorXi face_ids(unassociated_faces.size() + 1);
    face_ids[0] = -1;
    for(int i=0; i<unassociated_faces.size(); i++) {
      face_ids[i+1] = unassociated_faces[i]->getFaceId();
    }

    // create a tracker list for the probability table
    std::vector<TrackerStatusBayes::Ptr> trackers(tracker_status_map.size());
    std::transform(tracker_status_map.begin(), tracker_status_map.end(), trackers.begin(),
      [=](const std::pair<int, TrackerStatusBayes::Ptr>& p) { return p.second; }
    );

    // create tables of observation counts and probabilities
    auto tables = createCountProbabilityTables(trackers, unassociated_faces, face_ids);
    Eigen::MatrixXi& count_table = tables.first;
    Eigen::MatrixXd& prob_table = tables.second;

    for(const auto& fset: fmap) {
      int tracker_id = fset.first;
      auto tracker = std::find_if(trackers.begin(), trackers.end(), [=](const TrackerStatusBayes::Ptr& t) { return t->getTrackerId() == tracker_id; });
      if(tracker == trackers.end()) {
        continue;
      }

      int row = std::distance(trackers.begin(), tracker) + 1;
      for(int i=0; i<unassociated_faces.size(); i++) {
        std::vector<double> dists = unassociated_faces[i]->calcDistances(fset.second);
        for(double dist : dists) {
          Eigen::ArrayXd likelihoods = Eigen::ArrayXd::Ones(unassociated_faces.size() + 1);
          likelihoods[0] = neg_pdf_scale;
          likelihoods *= (*neg_pdf)(dist);
          likelihoods[i+1] = (*pos_pdf)(dist);

          prob_table.row(row).array() *= likelihoods;
        }
      }
      count_table.row(row).array() += fset.second.size();
    }


    shinkhornNormalization(prob_table);

    // update the probabilities of the faces and the trackers
    for(int i=0; i<unassociated_faces.size(); i++) {
      unassociated_faces[i]->setPosterior(prob_table(0, i+1));
    }
    for(int i=0; i<trackers.size(); i++) {
      trackers[i]->update(face_ids, count_table.row(i+1), prob_table.row(i+1));
    }

    // find associations to be established
    std::vector<std::tuple<int, int, int, double>> associations;  // row, col, count, posterior
    for(int i=1; i<prob_table.rows(); i++) {
      for(int j=0; j<prob_table.cols(); j++) {
        if(prob_table(i, j) > posterior_thresh && count_table(i, j) >= count_thresh) {
          associations.push_back(std::make_tuple(i, j, count_table(i, j), prob_table(i, j)));
        }
      }
    }

    std::cout << "--- count_table ---\n" << count_table << std::endl;
    std::cout << "--- prob_table ---\n" << prob_table << std::endl;

    // assign the face IDs to the trackers according to the associations
    for(const auto& assoc : associations) {
      std::cout << "(" << std::get<0>(assoc) << ", " << std::get<1>(assoc) << ") : " << std::get<2>(assoc) << " - " << std::get<3>(assoc) << std::endl;
      const auto& tracker = trackers[std::get<0>(assoc) - 1];
      if(std::get<1>(assoc) == 0) {
        std::cout << "new person!!" << std::endl;
        auto face = RegisteredFaceBayes::Ptr(new RegisteredFaceBayes(face_id_source++, tracker->getTrackerId()));
        associated_faces[tracker->getTrackerId()] = face;
      } else {
        std::cout << "known person!!" << std::endl;
        associated_faces[tracker->getTrackerId()] = unassociated_faces[std::get<1>(assoc) - 1];
        unassociated_faces.erase(unassociated_faces.begin() + std::get<1>(assoc) - 1);
      }
      auto& face = associated_faces[tracker->getTrackerId()];
      for(const auto& feature : tracker->getFeatures()) {
        face->addFace(*feature);
      }

      auto found = std::find_if(tracker_status_map.begin(), tracker_status_map.end(),
        [&](const std::pair<int, TrackerStatusBayes::Ptr>& p) { return p.second == tracker; }
      );
      if(found == tracker_status_map.end()){
        std::cout << "trackerstatusmapend!!" << std::endl;
        continue;
      }
      tracker_status_map.erase(found);
    }
  }

  /**
   * @brief updates the association of predefined faces
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

    std::cout << "updatedPredefined" << std::endl;
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
   * @param ids_msg   alive_ids
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

  /**
   * @brief sets predefined faces
   * @param predefined_faces  a set of name and image and feature vector of predefined faces
   */
  void setPredefinedFaces(const std::vector<std::tuple<std::string, cv::Mat, Eigen::VectorXf>>& predefined_faces) override {
    associated_predefined_faces.clear();
    unassociated_predefined_faces.clear();

    for(const auto& face : predefined_faces) {
      unassociated_predefined_faces.emplace_back();
      unassociated_predefined_faces.back().reset(new RegisteredFace(std::get<0>(face)));
      unassociated_predefined_faces.back()->addFace(std::get<2>(face));
    }
    std::cout << "setPredefinedFaces" << std::endl;
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
      RegisteredFaceBayes::Ptr new_face(new RegisteredFaceBayes(face->getFaceId()));
      new_face->addFaces(face);
      unassociated_faces.push_back(new_face);

      face_id_source = std::max(face_id_source, face->getFaceId() + 1);
    }
  }

  /**
   * @brief the set of registered faces
   * @return  the set of registered faces
   */
  std::vector<RegisteredFace::Ptr> getRegisteredFaces() const {
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
    neg_pdf_scale = config.neg_pdf_scale;
    count_thresh = config.num_observations_threshold;
    posterior_thresh = config.posterior_threshold;
  }

private:  
  std::pair<Eigen::MatrixXi, Eigen::MatrixXd> createCountProbabilityTables(const std::vector<TrackerStatusBayes::Ptr>& trackers, const std::vector<RegisteredFaceBayes::Ptr>& faces, const Eigen::VectorXi& face_ids) {
    std::pair<Eigen::MatrixXi, Eigen::MatrixXd> tables;
    Eigen::MatrixXi& count_table = tables.first;
    Eigen::MatrixXd& prob_table = tables.second;

    count_table = Eigen::MatrixXi::Zero(trackers.size() + 1, faces.size() + 1);
    prob_table = Eigen::MatrixXd::Zero(trackers.size() + 1, faces.size() + 1);

    for(int i=0; i<faces.size(); i++) {
      prob_table(0, i+1) = faces[i]->getPriori();
    }
    for(int i=0; i<trackers.size(); i++) {
      auto counts_priori = trackers[i]->createPrioriVector(face_ids);
      count_table.row(i+1) = counts_priori.first;
      prob_table.row(i+1) = counts_priori.second;
    }

    return tables;
  }

  void shinkhornNormalization(Eigen::MatrixXd& prob_table) const {
    bool converged = false;
    for(int i=0; i<10 && !converged; i++){
      converged = true;
      for(int i=1; i<prob_table.rows(); i++) {
        double sum = prob_table.row(i).sum();
        prob_table.row(i) /= sum;

        converged = converged & std::abs(1 - sum) < 1e-3;
      }
      for(int i=1; i<prob_table.cols(); i++) {
        double sum = prob_table.col(i).sum();
        prob_table.col(i) /= sum;

        converged = converged & std::abs(1 - sum) < 1e-3;
      }
    }
    prob_table = prob_table.array().max(1e-3).min(1.0 - 1e-3);
  }

  int face_id_source;
  std::unordered_map<int, TrackerStatusBayes::Ptr> tracker_status_map;

  std::vector<RegisteredFaceBayes::Ptr> unassociated_faces;
  std::unordered_map<int, RegisteredFaceBayes::Ptr> associated_faces;

  std::vector<RegisteredFace::Ptr> unassociated_predefined_faces;
  std::unordered_map<int, RegisteredFace::Ptr> associated_predefined_faces;

  double neg_pdf_scale;                             // the scaling parameter to model possibility of
  std::unique_ptr<SkewNormalDistribution> pos_pdf;  // the likelihood function for positive face pairs
  std::unique_ptr<SkewNormalDistribution> neg_pdf;  // the likelihood function for negative face pairs

  int count_thresh;           // the minimum number of observations for face_id establishment
  double posterior_thresh;    // the minimum posterior probability for face_id establishment
};

#endif // FACE_RECOGNIZER_BAYES_HPP

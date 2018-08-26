#ifndef TRACKER_STATUS_BAYES_HPP
#define TRACKER_STATUS_BAYES_HPP

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

#include "open_ptrack/recognition/registered_face.hpp"

/**
 * @brief the face recognition status of a tracker
 */
class TrackerStatusBayes {
public:
  typedef std::shared_ptr<TrackerStatusBayes> Ptr;

  TrackerStatusBayes(int tracker_id)
    : tracker_id(tracker_id),
      face_ids(1),
      counts(1),
      priori(1)
  {
    face_ids[0] = -1;
    counts[0] = 0;
    priori[0] = 0.1;
  }

  void addFace(const std::shared_ptr<Eigen::VectorXf>& feature) {
    features.push_back(feature);
  }

  int getTrackerId() const {
    return tracker_id;
  }

  const std::vector<std::shared_ptr<Eigen::VectorXf>>& getFeatures() const {
    return features;
  }

  std::pair<Eigen::VectorXi, Eigen::VectorXd> createPrioriVector(const Eigen::VectorXi& new_face_ids) {
    Eigen::VectorXi new_counts(new_face_ids.size());
    Eigen::VectorXd new_probs(new_face_ids.size());

    double eps = 0.1;

    int cursor = 0;
    for(int i=0; i<new_probs.size(); i++) {
      int id = new_face_ids[i];
      while(cursor != priori.size() && face_ids[cursor] < id) {
        cursor ++;
      }

      if(cursor == priori.size() || face_ids[cursor] != id) {
        new_probs[i] = eps;
        new_counts[i] = 0;
      } else {
        new_probs[i] = priori[cursor];
        new_counts[i] = counts[cursor];
      }
    }

    return std::make_pair(new_counts, new_probs);
  }

  void update(const Eigen::VectorXi& face_ids, const Eigen::VectorXi& counts, const Eigen::VectorXd& posterior) {
    this->face_ids = face_ids;
    this->counts = counts;
    this->priori = posterior;
  }

private:
  int tracker_id;
  std::vector<std::shared_ptr<Eigen::VectorXf>> features;

  Eigen::VectorXi face_ids;   // IDs of the faces
  Eigen::VectorXi counts;     // the numbers of observations for each face
  Eigen::VectorXd priori;     // the priori probabilities for each face
};

#endif // BAYES_INFERENCE_STATUS_HPP

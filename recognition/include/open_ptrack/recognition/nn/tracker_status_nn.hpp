#ifndef TRACKER_STATUS_NN_HPP
#define TRACKER_STATUS_NN_HPP

#include <memory>
#include <vector>
#include "open_ptrack/recognition/registered_face.hpp"

/**
 * @brief the recognition status of a tracker
 */
class TrackerStatusNN {
public:
  typedef std::shared_ptr<TrackerStatusNN> Ptr;

  /**
   * @brief constructor
   * @param tracker_id  ID of the tracker
   */
  TrackerStatusNN(int tracker_id)
    : tracker_id(tracker_id)
  {
  }

  /**
   * @brief adds a face recognition result to the results list
   * @param feature  the recognized face feature vector
   * @param result   the recognition result
   */
  void addResult(const std::shared_ptr<Eigen::VectorXf>& feature, RegisteredFace::Ptr result) {
    features.push_back(feature);
    results.push_back(result);
  }

  int getTrackerId() const {
    return tracker_id;
  }

  const std::vector<std::shared_ptr<Eigen::VectorXf>>& getFeatures() const {
    return features;
  }

  const std::vector<RegisteredFace::Ptr>& getResults() const {
    return results;
  }

private:
  int tracker_id;

  std::vector<RegisteredFace::Ptr> results;                 //
  std::vector<std::shared_ptr<Eigen::VectorXf>> features;   //
};

#endif // FACE_RECOGNITION_STATUS_HPP

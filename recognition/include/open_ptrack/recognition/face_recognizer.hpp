#ifndef FACE_RECOGNIZER_HPP
#define FACE_RECOGNIZER_HPP

#include <memory>
#include <iostream>
#include <unordered_map>
#include <opt_msgs/IDArray.h>
#include <opt_msgs/TrackArray.h>
#include <recognition/FaceRecognitionConfig.h>

#include "open_ptrack/recognition/registered_face.hpp"

class FaceRecognizer {
public:

  FaceRecognizer() {}
  virtual ~FaceRecognizer() {}

  /**
   * @brief updates the face recognizer
   * @param tracker_id  the id given by the tracker
   * @param features    the face feature vectors
   */
  virtual void update(const std::unordered_map<int, std::vector<std::shared_ptr<Eigen::VectorXf>>>&) = 0;

  /**
   * @brief updates the association of predefined faces
   */
  virtual void updatePredefined() = 0;


  /**
   * @brief converts a tracker ID to a face recognition ID
   * @param tracker_id  the tracker ID
   * @return the face recognition ID
   */
  virtual int convertID(int tracker_id) const = 0;

  /**
   * @brief disposes unnecessary data
   * @param ids_msg   alive_ids
   */
  virtual void collectGarbage(const opt_msgs::TrackArrayConstPtr& trackers) = 0;

  /**
   * @brief sets predefined faces
   * @param predefined_faces  a set of name and image and feature vector of predefined faces
   */
  virtual void setPredefinedFaces(const std::vector<std::tuple<std::string, cv::Mat, Eigen::VectorXf>>& predefined_faces) = 0;

  /**
   * @brief the set of predefined faces associated with the registered faces
   * @return  the set of predefined faces
   */
  virtual const std::unordered_map<int, RegisteredFace::Ptr>& getAssociatedPredefinedFaces() const = 0;

  /**
   * @brief registers faces to the face recognizer
   * @param faces  faces to be registered
   */
  virtual void registerFaces(const std::vector<RegisteredFace::Ptr>& faces) = 0;

  /**
   * @brief the set of registered faces
   * @return  the set of registered faces
   */
  virtual std::vector<RegisteredFace::Ptr> getRegisteredFaces() const = 0;

  /**
   * @brief callback for dynamic reconfigure
   * @param config  configuration parameters
   * @param level   configuration level
   */
  virtual void cfgCallback(recognition::FaceRecognitionConfig& config, uint32_t level) = 0;
};

#endif // FACE_RECOGNIZER_HPP

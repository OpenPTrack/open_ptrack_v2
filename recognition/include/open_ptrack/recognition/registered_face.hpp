#ifndef REGISTERED_FACE_HPP
#define REGISTERED_FACE_HPP

#include <memory>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief face registered to the recognition system
 *        this class consists of a set of face features ,face_id, tracker_id, name of the registered face
 */
class RegisteredFace {
public:
  using Ptr = std::shared_ptr<RegisteredFace>;

  /**
   * @brief constructor
   * @param face_id      id of the face
   * @param tracker_id   id of the tracker associated with this face (-1 means no tracker is associated)
   */
  RegisteredFace(int face_id, int tracker_id = -1)
    : face_id (face_id),
      tracker_id (tracker_id)
  {
    faces.reserve(32);
  }

  /**
   * @brief constructor
   * @param name    the name of the person
   */
  RegisteredFace(const std::string& name)
    : face_id(-1),
      tracker_id(-1),
      name(name)
  {
    faces.reserve(32);
  }

  virtual ~RegisteredFace() {}

  /**
   * @brief adds a face feature to the feature list
   * @param face  feature vector to be added
   */
  void addFace(const Eigen::VectorXf& face) {
    faces.push_back(face);
  }

  /**
   * @brief adds a set of face features to the feature list
   * @param faces  features to be added
   */
  void addFaces(const RegisteredFace::Ptr& faces) {
    for(const auto& face : faces->faces) {
      addFace(face);
    }
  }

  /**
   * @brief calculates the distance between #face and the closest face in the feature list
   * @param face  face feature vector
   * @return the distance between #face and the closest face
   */
  double calcDistance(const Eigen::VectorXf& face) const {
    double dist = std::numeric_limits<double>::max();
    for(const auto& f : faces) {
      if(f.rows() == face.rows() && f.cols() == face.cols()){
        dist = std::min<double>( dist, (f - face).squaredNorm() );
      }
      else{
        std::cout << "row/col comparison failed" << std::endl;
        return 0;
      }
    }
    return std::sqrt(dist);
  }

  /**
   * @brief calculates the distance between #face and the closest face in the feature list
   * @param face  face feature vector
   * @return the distance between #face and the closest face
   */
  double calcDistance(const Eigen::VectorXf& face, int n) const {
    std::vector<double> dists(faces.size());
    std::transform(faces.begin(), faces.end(), dists.begin(), [&](const Eigen::VectorXf& f) { return (f - face).norm(); });
    std::sort(dists.begin(), dists.end());

    n = std::min<int>(n, dists.size());
    double accum = std::accumulate(dists.begin(), dists.begin() + n, 0.0);
    return accum / n;
  }

  /**
   * @brief calculates the distances
   * @param features
   * @return
   */
  std::vector<double> calcDistances(const std::vector<std::shared_ptr<Eigen::VectorXf>>& features) const {
    std::vector<double> dists(features.size());
    std::transform(features.begin(), features.end(), dists.begin(),
      [&](const std::shared_ptr<Eigen::VectorXf>& f) {
        return calcDistance(*f);
      }
    );

    return dists;
  }

  /**
   * @brief calculates the distances
   * @param features
   * @return
   */
  std::vector<double> calcDistances(const std::vector<std::shared_ptr<Eigen::VectorXf>>& features, int n) const {
    std::vector<double> dists(features.size());
    std::transform(features.begin(), features.end(), dists.begin(),
      [&](const std::shared_ptr<Eigen::VectorXf>& f) {
        return calcDistance(*f, n);
      }
    );

    return dists;
  }

  /**
   * @brief calculates the average distance between features in #rhs and the n closest neighbors in the feature list
   * @param rhs  feature vectors
   * @param n    number of the neighbors
   * @return the distance
   */
  double calcDistance(const RegisteredFace& rhs, int n) const {
    std::vector<double> dists(rhs.size());
    std::transform(rhs.faces.begin(), rhs.faces.end(), dists.begin(), [&](const Eigen::VectorXf& v){ return calcDistance(v);});

    std::sort(dists.begin(), dists.end());

    n = std::min<int>(n, dists.size());
    double accum = std::accumulate(dists.begin(), dists.begin() + n, 0.0);
    return accum / n;
  }

  /**
   * @brief the number of face features in the list
   * @return
   */
  int size() const {
    return faces.size();
  }

  int getFaceId() const {
    return face_id;
  }

  void setTrackerId(int id) {
    tracker_id = id;
  }

  int getTrackerId() const {
    return tracker_id;
  }

  const std::string& getName() const {
    return name;
  }

  std::vector<Eigen::VectorXf>& getFaces() {
    return faces;
  }

  const std::vector<Eigen::VectorXf>& getFaces() const {
    return faces;
  }

private:
  int face_id;        // ID of the face
  int tracker_id;     // ID of the tracker associated with the face (-1 means no tracker is associated)
  std::string name;   // name of the person

  std::vector<Eigen::VectorXf> faces;   // face feature list
};



#endif // FACE_DATA_HPP

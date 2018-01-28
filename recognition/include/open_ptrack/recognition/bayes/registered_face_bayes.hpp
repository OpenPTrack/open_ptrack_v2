#ifndef REGISTERED_FACE_BAYES_HPP
#define REGISTERED_FACE_BAYES_HPP

#include "open_ptrack/recognition/registered_face.hpp"

class RegisteredFaceBayes : public RegisteredFace {
public:
  using Ptr = std::shared_ptr<RegisteredFaceBayes>;

  RegisteredFaceBayes(int face_id, int tracker_id = -1)
    : RegisteredFace(face_id, tracker_id)
  {
    priori = 0.1;
  }

  void setPosterior(double p) {
    priori = p;
  }

  double getPriori() const {
    return priori;
  }

private:
  double priori;
};

#endif // REGISTERED_FACE_BAYES_HPP

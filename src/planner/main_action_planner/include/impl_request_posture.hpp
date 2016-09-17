#ifndef __IMPL_REQUEST_POSTURE_H_INCLUDE__
#define __IMPL_REQUEST_POSTURE_H_INCLUDE__

#include "request_posture.hpp"

class PatternRequestPosture : public RequestPosture {
public:
  PatternRequestPosture();
  virtual void requestPosture(std::vector<double>& posture);
};

class TrajectoryRequestPosture : public RequestPosture {
public:
  TrajectoryRequestPosture();
  virtual void requestPosture(std::vector<double>& posture);
};

#endif


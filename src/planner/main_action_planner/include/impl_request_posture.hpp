#ifndef __IMPL_REQUEST_POSTURE_H_INCLUDE__
#define __IMPL_REQUEST_POSTURE_H_INCLUDE__

#include "request_posture.hpp"

class PatternRequestPosture : RequestPosture {
public:
  virtual ~PatternRequestPosture();
  virtual void requestPosture(std::vector<double>& posture);
};

class TrajectoryRequestPosture : RequestPosture {
public:
  virtual ~TrajectoryRequestPosture();
  virtual void requestPosture(std::vector<double>& posture);
};

#endif


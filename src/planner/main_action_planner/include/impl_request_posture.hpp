#ifndef __IMPL_REQUEST_POSTURE_H_INCLUDE__
#define __IMPL_REQUEST_POSTURE_H_INCLUDE__

#include "request_posture.hpp"

class PatternRequestPosture : public RequestPosture {
public:
  PatternRequestPosture(ros::NodeHandle& nh);
  virtual void requestPosture(std::vector<double>& posture);
private:
  ros::NodeHandle nh;
};

class TrajectoryRequestPosture : public RequestPosture {
public:
  TrajectoryRequestPosture(ros::NodeHandle& nh);
  virtual void requestPosture(std::vector<double>& posture);
private:
  ros::NodeHandle nh;
};

#endif


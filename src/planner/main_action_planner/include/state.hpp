#ifndef __STATE_CHECKER_H_INCLUDE__
#define __STATE_CHECKER_H_INCLUDE__

#include <map>

#include "ros/ros.h"
#include "state_msgs/State.h"

class StateChecker {
public:
  static const std::string FOUND_TARGET;
  static const std::string NEAR_TARGET;
  static const std::string HOLD;
  static const std::string CAN_RELEASE;

  StateChecker(ros::NodeHandle&);
  bool get(std::string);
private:
  void update(const state_msgs::State::ConstPtr&);
  std::map<std::string, bool> status;
  ros::Subscriber sub;
};

#endif


#ifndef __STATE_CHECKER_H_INCLUDE__
#define __STATE_CHECKER_H_INCLUDE__

#include <map>
#include <vector>

#include "ros/ros.h"
#include "state_msgs/State.h"

class StateChecker {
public:
  static const std::string FOUND_TARGET;
  static const std::string NEAR_TARGET;
  static const std::string HOLD;
  static const std::string CAN_RELEASE;

  StateChecker(ros::NodeHandle&);
  std::vector<std::string> getPostureKey() noexcept;
private:
  void update(const state_msgs::State::ConstPtr&);
  std::map<std::string, bool> status;
  ros::Subscriber sub;
};

#endif


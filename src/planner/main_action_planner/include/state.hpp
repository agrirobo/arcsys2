#ifndef __STATE_CHECKER_H_INCLUDE__
#define __STATE_CHECKER_H_INCLUDE__

#include <map>

#include "ros/ros.h"
#include "state_pub/States.h"

class StateChecker {
public:
  enum class States : std::string {
    FOUND_TARGET = "foundTarget",
    NEAR_TARGET = "nearTarget",
    HOLD = "hold",
    CAN_RELEASE = "canRelease"
  };

  StateChecker(ros::NodeHandle&);
  bool get(States);
private:
  void update(state_msgs::State::ConstPtr&);
  std::map<States, bool> status;
  ros::Subscriber sub;
};

#endif


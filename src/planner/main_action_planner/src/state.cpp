#include "state.hpp"

const std::string StateChecker::FOUND_TARGET = "foundTarget";
const std::string StateChecker::NEAR_TARGET = "nearTarget";
const std::string StateChecker::HOLD = "hold";
const std::string StateChecker::CAN_RELEASE = "canRelease";

StateChecker::StateChecker(ros::NodeHandle& n)
: sub(n.subscribe("/robot_status", 5, &StateChecker::update, this))
{}

bool StateChecker::get(std::string state) {
  return status[state];
}

void StateChecker::update(const state_msgs::State::ConstPtr& msg) {
  status[msg->name] = msg->state;
}

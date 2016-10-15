#include "state.hpp"

StateChecker::StateChecker(ros::NodeHandle& n)
: sub(n.subscribe("/robot_status", 5, update, this))
{}

bool StateChecker::get(States state) {
  return status[state];
}

void StateChecker::update(state_msgs::State::ConstPtr& msg) {
  status[static_cast<States>(msg->name)] = msg->state;
}

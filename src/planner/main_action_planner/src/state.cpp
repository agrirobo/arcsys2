#include"state.hpp"

#include"request_posture.hpp"

const std::string StateChecker::FOUND_TARGET = "foundTarget";
const std::string StateChecker::HOLD = "hold";
const std::string StateChecker::NEAR_TARGET = "nearTarget";
const std::string StateChecker::CAN_RELEASE = "canRelease";

StateChecker::StateChecker(ros::NodeHandle& n)
: sub(n.subscribe("/robot_status", 5, &StateChecker::update, this))
{}

void StateChecker::update(const state_msgs::State::ConstPtr& msg) {
  status[msg->name] = msg->state;
}

std::vector<std::string> StateChecker::getPostureKey() noexcept{
  if (status[CAN_RELEASE]) // 離すべきタイミング
    return {RequestPostureFactory::release, "hand"};
  if (status[HOLD]) // 保持中
    return {RequestPostureFactory::move2release, "krs"};
  if (status[NEAR_TARGET]) // 収穫動作に入るか
    return {RequestPostureFactory::harvest, "hand"};
  if (status[FOUND_TARGET]) // 見つかっているか
    return {RequestPostureFactory::move2face, "base"};
  return {RequestPostureFactory::move2find, "base"};
}

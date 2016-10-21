#include "impl_request_posture.hpp"

#include "pattern_posture_generator/PatternPosture.h"
const std::string RequestPostureFactory::release = "release";
const std::string RequestPostureFactory::move2release = "pattern_relese";
const std::string RequestPostureFactory::harvest = "harvest";
const std::string RequestPostureFactory::ik = "torajectory";
const std::string RequestPostureFactory::move_vertical = "vertical";
const std::string RequestPostureFactory::move2face = "base_face";
const std::string RequestPostureFactory::move2find = "base_find";

RequestPostureFactory RequestPostureFactory::unique;

RequestPostureFactory::RequestPostureFactory() : reqs() {}

RequestPostureFactory::~RequestPostureFactory() {
  for (std::map<std::string, RequestPosture*>::iterator it = reqs.begin(), end_it = reqs.end();
       it != end_it;
       it++) {
    delete it->second;
    it->second = NULL;
  }
}

RequestPosture* RequestPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  const auto& found_it = unique.reqs.find(name);
  if (found_it == unique.reqs.end()) return unique.create(name, nh);
  return found_it->second;
}

RequestPosture* RequestPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  if (name == move2release) return reqs[move2release] = new PatternRequestPosture {nh, "release"};
  if (name == ik) return reqs[ik] = new TrajectoryRequestPosture {nh};
  return nullptr;
}

PatternRequestPosture::PatternRequestPosture(ros::NodeHandle& nh, std::string state)
: state {state},
  client {nh.serviceClient<pattern_posture_generator::PatternPosture>("getPosture")}
{}

PatternRequestPosture::PatternRequestPosture(ros::NodeHandle& nh)
: state {"normal"},
  client {nh.serviceClient<pattern_posture_generator::PatternPosture>("getPosture")}
{}

void PatternRequestPosture::requestPosture(std::vector<double>& posture) {
  pattern_posture_generator::PatternPosture srv;
  srv.request.name = state;
  if (client.call(srv)) {
    ROS_INFO("Error call: No response by Request [%s] PatternPosture", state.c_str());
    return;
  }
  std::copy(srv.response.posture.begin(), srv.response.posture.end(), std::back_inserter(posture));
}

TrajectoryRequestPosture::TrajectoryRequestPosture(ros::NodeHandle& nh) {

}

void TrajectoryRequestPosture::requestPosture(std::vector<double>& posture) {

}


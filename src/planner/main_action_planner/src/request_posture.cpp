#include "impl_request_posture.hpp"

#include "pattern_posture_generator/PatternPosture.h"

std::map<std::string, RequestPosture*> RequestPostureFactory::reqs;

RequestPosture* RequestPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  std::map<std::string, RequestPosture*>::const_iterator found_it = reqs.find(name);
  if (found_it == reqs.end()) return create(name, nh);
  return found_it->second;
}

RequestPosture* RequestPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  if (name == "pattern") {
    reqs["pattern"] = new PatternRequestPosture(nh);
    return reqs.at("pattern");
  }
  if (name == "torajectory") {
    reqs["torajectory"] = new TrajectoryRequestPosture(nh);
    return reqs.at("torajectory");
  }
  return NULL;
}

PatternRequestPosture::PatternRequestPosture(ros::NodeHandle& nh, std::string state) : state(state) {
  client = nh.serviceClient<pattern_posture_generator::PatternPosture>("/getPosture");
}

PatternRequestPosture::PatternRequestPosture(ros::NodeHandle& nh) : state("normal") {
  client = nh.serviceClient<pattern_posture_generator::PatternPosture>("/getPosture");
}

void PatternRequestPosture::requestPosture(std::vector<double>& posture) {
  
}

TrajectoryRequestPosture::TrajectoryRequestPosture(ros::NodeHandle& nh) {

}

void TrajectoryRequestPosture::requestPosture(std::vector<double>& posture) {

}


#include "impl_request_posture.hpp"

std::map<std::string, RequestPosture*> RequestPostureFactory::reqs;

const RequestPosture* RequestPostureFactory::get(const std::string& name) {
  std::map<std::string, RequestPosture*>::const_iterator found_it = reqs.find(name);
  if (found_it != reqs.end()) return create(name);
  return found_it->second;
}

const RequestPosture* RequestPostureFactory::create(const std::string& name) {
  if (name == "pattern") {
    reqs["pattern"] = new PatternRequestPosture;
    return reqs.at("pattern");
  }
  if (name == "torajectory") {
    reqs["torajectory"] = new TrajectoryRequestPosture;
    return reqs.at("torajectory");
  }
  return NULL;
}

PatternRequestPosture::PatternRequestPosture() {
  
}

void PatternRequestPosture::requestPosture(std::vector<double>& posture) {
  
}

TrajectoryRequestPosture::TrajectoryRequestPosture() {

}

void TrajectoryRequestPosture::requestPosture(std::vector<double>& posture) {

}


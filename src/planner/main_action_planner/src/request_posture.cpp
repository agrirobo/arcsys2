#include "impl_request_posture.hpp"

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
    reqs["torajectory"] = new TorajectoryReqestPosture;
    return reqs.at("torajectory");
  }
}


#include "impl_request_posture.hpp"

static RequsetPosture* pattern = new PatternRequestPosture;
static RequestPosture* torajectoy = new TorajectoryRequestPosture;

std::map<std::string, RequestPosture*> RequestPostureFactory::reqs = {{"pattern", pattern},
                                                                      {"torajectory", torajectory}};

const RequestPosture* RequestPostureFactory::get(std::string& name) {
  std::map<std::string, RequestPosture*>::iterator found_it = reqs.find(name);
  if (found_it != reqs.end()) return NULL;
  return found_it;
}

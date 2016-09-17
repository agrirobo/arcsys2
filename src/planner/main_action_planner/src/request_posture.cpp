#include "impl_request_posture.hpp"

std::map<std::string, RequestPosture*> RequestPostureFactory::reqs;

const RequestPosture* RequestPostureFactory::get(std::string& name) {
  std::map<std::string, RequestPosture*>::const_iterator found_it = reqs.find(name);
  if (found_it != reqs.end()) return NULL;
  return found_it->second;
}


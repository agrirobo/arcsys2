#include "impl_request_posture.hpp"

std::map<std::string, const SendPosture*> SendPostureFactory::sends;

const SendPosture* SendPostureFactory::get(const std::string& name) {
  std::map<std::string, const SendPosture*>::const_iterator found_it = sends.find(name);
  if (found_it != sends.end()) return create(name);
  return found_it->second;
}

const SendPosture* SendPostureFactory::create(const std::string& name) {
  if (name == "krs") {
    sends["krs"] = new KrsSendPosture;
    return sends.at("krs");
  }
  return NULL;
}

KrsSendPosture::KrsSendPosture() {
  
}

void KrsSendPosture::sendPosture(std::vector<double>& posture) {
  
}


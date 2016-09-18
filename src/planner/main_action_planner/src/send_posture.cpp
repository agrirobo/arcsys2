#include "impl_send_posture.hpp"

std::map<std::string, const SendPosture*> SendPostureFactory::sends;

const SendPosture* SendPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  std::map<std::string, const SendPosture*>::const_iterator found_it = sends.find(name);
  if (found_it != sends.end()) return create(name, nh);
  return found_it->second;
}

const SendPosture* SendPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  if (name == "krs") {
    sends["krs"] = new KrsSendPosture(nh);
    return sends.at("krs");
  }
  return NULL;
}

KrsSendPosture::KrsSendPosture(ros::NodeHandle& nh) {
  
}

void KrsSendPosture::sendPosture(std::vector<double>& posture) {
  
}


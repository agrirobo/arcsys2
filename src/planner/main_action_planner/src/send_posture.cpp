#include "impl_send_posture.hpp"

#include<algorithm>

SendPostureFactory SendPostureFactory::unique;

SendPostureFactory::SendPostureFactory() : sends() {}

SendPostureFactory::~SendPostureFactory() {
  for (auto it = sends.begin(), end_it = sends.end();
       it != end_it;
       it++) {
    delete it->second;
    it->second = NULL;
  }
}

SendPosture* SendPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  const auto& found_it = unique.sends.find(name);
  if (found_it == unique.sends.end()) return unique.create(name, nh);
  return found_it->second;
}

SendPosture* SendPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  return NULL;
}

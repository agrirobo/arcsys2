#include "impl_request_posture.hpp"

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
  for (auto it = reqs.begin(), end_it = reqs.end();
       it != end_it;
       it++) {
    delete it->second;
    it->second = nullptr;
  }
}

RequestPosture* RequestPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  const auto& found_it = unique.reqs.find(name);
  if (found_it == unique.reqs.end()) return unique.create(name, nh);
  return found_it->second;
}

RequestPosture* RequestPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  return nullptr;
}

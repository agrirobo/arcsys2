#include "ros/ros.h"

#include "pattern_posture_generator_node.hpp"

PatternPostureGenerator::PatternPostureGenerator(){}

PatternPostureGenerator::PatternPostureGenerator(ros::NodeHandle& nh) {
  if (!nh.getParam("pattern", pattern_names)) return;
  for (std::map<std::string, std::string >::iterator it = pattern_names.begin();
       it != pattern_names.end();
       it++) {
    std::vector<double> posture;
    if (!nh.getParam(std::string("pattern/").append(it->second), posture)) return;
    std::copy(posture.begin(), posture.end(), std::back_inserter(posture_datas[it->second]));
  }
  key_srv = nh.advertiseService("getPostureKey", &PatternPostureGenerator::getPostureKey, this);
}

bool PatternPostureGenerator::getPostureKey(pattern_posture_generator::PatternKeyPosture::Request&  req,
                                            pattern_posture_generator::PatternKeyPosture::Response& res) {
  return true;
}

int main(int argc, char* argv[]) {
  return 0;
}

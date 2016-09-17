#include "ros/ros.h"

#include "pattern_posture_generator_node.hpp"

void dumpMap(std::map<std::string, std::string>& map) {
  std::string map_info;
  for (std::map<std::string, std::string>::iterator it = map.begin(), end_it = map.end();
       it != end_it;
       it++) {
    map_info.append(it->second);
    map_info.push_back(':');
  }
  ROS_INFO("%s", map_info.c_str());
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pattern_posture_generator");
  ros::NodeHandle nh;

  PatternPostureGenerator ppg(nh);

  ROS_INFO("Ready. getPostureKey");
  ros::spin();
  return 0;
}

PatternPostureGenerator::PatternPostureGenerator(){}

PatternPostureGenerator::PatternPostureGenerator(ros::NodeHandle& nh) {
  if (!nh.hasParam("pattern")) throw;
  nh.getParam("pattern", pattern_names);
  ROS_INFO("Get map of patterns parent");
  dumpMap(pattern_names);
  for (std::map<std::string, std::string>::iterator it = pattern_names.begin(), end_it = pattern_names.end();
       it != end_it;
       it++) {
    std::vector<double> posture;
    std::string posture_param_name = std::string("pattern/").append(it->second);
    nh.getParam(posture_param_name, posture);
    std::copy(posture.begin(), posture.end(), std::back_inserter(posture_datas[it->second]));
    ROS_INFO("Found posture of [%s]", it->second.c_str());
  }
  key_srv = nh.advertiseService("getPostureKey", &PatternPostureGenerator::getPostureKey, this);
}

bool PatternPostureGenerator::getPostureKey(pattern_posture_generator::PatternKeyPosture::Request&  req,
                                            pattern_posture_generator::PatternKeyPosture::Response& res) {
  std::copy(posture_datas[req.name].begin(), posture_datas[req.name].end(), std::back_inserter(res.posture));
  return true;
}

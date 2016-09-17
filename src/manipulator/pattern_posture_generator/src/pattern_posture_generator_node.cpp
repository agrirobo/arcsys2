#include "pattern_posture_generator_node.hpp"

#include <sstream>

void dumpMap(std::map<std::string, double>& map) {
  std::stringstream map_info;
  for (std::map<std::string, double>::iterator it = map.begin(), end_it = map.end();
       it != end_it;
       it++)
    map_info << it->second << ':';
  ROS_INFO("%s", map_info.str().c_str());
}

void dumpVector(std::vector<std::string> vec) {
  std::string vec_info;
  for (std::vector<std::string>::iterator it = vec.begin(), end_it = vec.end();
       it != end_it;
       it++) {
    vec_info.append(*it);
    vec_info.push_back(':');
  }
  ROS_INFO("%s", vec_info.c_str());
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

PatternPostureGenerator::PatternPostureGenerator(ros::NodeHandle& nh) : nh(nh) {
  key_srv = nh.advertiseService("getPostureKey", &PatternPostureGenerator::getPostureKey, this);
  reload_srv = nh.advertiseService("reload", &PatternPostureGenerator::reload, this);
  reload();
}

bool PatternPostureGenerator::reload() {
  if (!nh.hasParam("pattern_names")) return false;
  pattern_names.clear();
  nh.getParam("pattern_names", pattern_names);
  dumpVector(pattern_names);
  posture_datas.clear();
  for (std::vector<std::string>::iterator it = pattern_names.begin(), end_it = pattern_names.end();
       it != end_it;
       it++) {
    if (!nh.hasParam(*it)) continue;
    std::vector<double> posture;
    nh.getParam(*it, posture);
    std::copy(posture.begin(), posture.end(), std::back_inserter(posture_datas[*it]));
    ROS_INFO("Found posture of [%s]", it->c_str());
  }
  return true;
}

bool PatternPostureGenerator::reload(std_srvs::Empty::Request&  req,
                                     std_srvs::Empty::Response& res) {
  return reload();
}

bool PatternPostureGenerator::getPostureKey(pattern_posture_generator::PatternKeyPosture::Request&  req,
                                            pattern_posture_generator::PatternKeyPosture::Response& res) {
  if (posture_datas.find(req.name) == posture_datas.end()) return false;
  std::copy(posture_datas[req.name].begin(), posture_datas[req.name].end(), std::back_inserter(res.posture));
  return true;
}

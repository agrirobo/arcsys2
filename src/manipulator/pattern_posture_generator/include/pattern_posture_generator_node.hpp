#ifndef __PATTERN_POSTURE_GENERATOR_NODE_H_INCLUDE__
#define __PATTERN_POSTURE_GENERATOR_NODE_H_INCLUDE__

#include "ros/ros.h"
#include "pattern_posture_generator/PatternPosture.h"
#include "std_srvs/Empty.h"

#include <vector>
#include <map>

class PatternPostureGenerator {
private:
  PatternPostureGenerator();
public:
  explicit PatternPostureGenerator(ros::NodeHandle& nh);

private:
  bool getPosture(pattern_posture_generator::PatternPosture::Request&  req,
                  pattern_posture_generator::PatternPosture::Response& res);
  bool reload();
  bool reload(std_srvs::Empty::Request&  req,
              std_srvs::Empty::Response& res);

private:
  ros::NodeHandle nh;
  ros::ServiceServer reload_srv;
  ros::ServiceServer key_srv;
  std::map<std::string, std::vector<double> > posture_datas;
};

#endif

#ifndef __PATTERN_POSTURE_GENERATOR_NODE_H_INCLUDE__
#define __PATTERN_POSTURE_GENERATOR_NODE_H_INCLUDE__

#include <vector>
#include <map>

#include "pattern_posture_generator/PatternKeyPosture.h"
#include "std_srvs/Empty.h"

class PatternPostureGenerator {
private:
  PatternPostureGenerator();
public:
  explicit PatternPostureGenerator(ros::NodeHandle& nh);

private:
  bool getPostureKey(pattern_posture_generator::PatternKeyPosture::Request&  req,
                     pattern_posture_generator::PatternKeyPosture::Response& res);
  bool reload(std_srvs::Empty::Request&  req,
              std_srvs::Empty::Response& res);

private:
  ros::NodeHandle nh;
  ros::ServiceServer reload_srv;
  ros::ServiceServer key_srv;
  std::vector<std::string> pattern_names;
  std::map<std::string, std::vector<double> > posture_datas;
};

#endif

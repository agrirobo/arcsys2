#ifndef __REQUEST_POSTURE_H_INCLUDE__
#define __REQUEST_POSTURE_H_INCLUDE__

#include "ros/ros.h"

#include <vector>
#include <string>
#include <map>

class RequestPosture {
public:
  virtual ~RequestPosture(){};
  virtual void requestPosture(std::vector<double>& posture) = 0;
};

class RequestPostureFactory {
private:
  RequestPostureFactory();

public:
  static const RequestPosture* get(const std::string& name, ros::NodeHandle& nh);

private:
  static const RequestPosture* create(const std::string& name, ros::NodeHandle& nh);
  static std::map<std::string, const RequestPosture*> reqs;
};

#endif


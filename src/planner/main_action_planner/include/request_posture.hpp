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
public:
  static RequestPosture* get(const std::string& name, ros::NodeHandle& nh);

private:
  RequestPostureFactory();
  RequestPostureFactory(const RequestPostureFactory& other);
  ~RequestPostureFactory();
  RequestPostureFactory& operator=(const RequestPostureFactory& other);

  RequestPosture* create(const std::string& name, ros::NodeHandle& nh);

  std::map<std::string, RequestPosture*> reqs;

  static RequestPostureFactory unique;
};

#endif


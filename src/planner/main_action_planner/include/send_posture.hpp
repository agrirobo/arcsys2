#ifndef __SEND_POSTURE_H_INCLUDE__
#define __SEND_POSTURE_H_INCLUDE__

#include "ros/ros.h"

#include <vector>
#include <string>
#include <map>

class SendPosture {
public:
  virtual ~SendPosture(){};
  virtual void sendPosture(std::vector<double>& posture) = 0;
};

class SendPostureFactory {
private:
  SendPostureFactory();

public:
  static SendPosture* get(const std::string& name, ros::NodeHandle& nh);

private:
  static SendPosture* create(const std::string& name, ros::NodeHandle& nh);
  static std::map<std::string, SendPosture*> sends;
};

#endif


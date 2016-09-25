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
public:
  SendPosture* get(const std::string& name, ros::NodeHandle& nh);

private:
  SendPostureFactory();
  SendPostureFactory(const SendPostureFactory& other);
  ~SendPostureFactory();
  SendPostureFactory& operator=(const SendPostureFactory& other);

  SendPosture* create(const std::string& name, ros::NodeHandle& nh);

  std::map<std::string, SendPosture*> sends;

  static SendPostureFactory unique;
};

#endif


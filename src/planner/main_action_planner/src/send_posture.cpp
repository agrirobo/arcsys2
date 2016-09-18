#include "impl_send_posture.hpp"

#include "algorithm"

#include "servo_msgs/KrsServoDegree.h"

std::map<std::string, const SendPosture*> SendPostureFactory::sends;

const SendPosture* SendPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  std::map<std::string, const SendPosture*>::const_iterator found_it = sends.find(name);
  if (found_it != sends.end()) return create(name, nh);
  return found_it->second;
}

const SendPosture* SendPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  if (name == "krs") {
    sends["krs"] = new KrsSendPosture(nh);
    return sends.at("krs");
  }
  return NULL;
}

KrsSendPosture::KrsSendPosture(ros::NodeHandle& nh) : nh(nh) {
  pub = nh.advertise<servo_msgs::KrsServoDegree>("cmd_krs", 16);
  nh.getParam("id_vec", id_vec);
}

void KrsSendPosture::sendPosture(std::vector<double>& posture) {
  servo_msgs::KrsServoDegree msg;
  std::vector<double>::size_type length = std::min(posture.size(), id_vec.size());
  for (std::vector<double>::size_type i = 0; i < length; i++) {
    msg.id = id_vec[i];
    msg.angle = posture[i];
    pub.publish(msg);
  }
}


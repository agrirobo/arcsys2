#include "impl_send_posture.hpp"

#include "algorithm"

#include "servo_msgs/KrsServoDegree.h"
#include "std_srvs/Empty.h"

SendPostureFactory SendPostureFactory::unique;

SendPostureFactory::SendPostureFactory() : sends() {}

SendPostureFactory::~SendPostureFactory() {
  for (std::map<std::string, SendPosture*>::iterator it = sends.begin(), end_it = sends.end();
       it != end_it;
       it++) {
    delete it->second;
    it->second = NULL;
  }
}

SendPosture* SendPostureFactory::get(const std::string& name, ros::NodeHandle& nh) {
  std::map<std::string, SendPosture*>::const_iterator found_it = unique.sends.find(name);
  if (found_it == unique.sends.end()) return unique.create(name, nh);
  return found_it->second;
}

SendPosture* SendPostureFactory::create(const std::string& name, ros::NodeHandle& nh) {
  if (name == "krs") {
    sends["krs"] = new KrsSendPosture(nh);
    return sends.at("krs");
  }
  return NULL;
}

KrsSendPosture::KrsSendPosture(ros::NodeHandle& nh) : nh(nh) {
  pub = nh.advertise<servo_msgs::KrsServoDegree>("cmd_krs", 16);
  reload_srv = nh.advertiseService("reload_id_vec", &KrsSendPosture::reload, this);
  reload();
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

bool KrsSendPosture::reload() {
  if (!nh.hasParam("id_vec")) return false;
  id_vec.clear();
  nh.getParam("id_vec", id_vec);
  return true;
}

bool KrsSendPosture::reload(std_srvs::Empty::Request&  req,
                            std_srvs::Empty::Response& res) {
  return reload();
}

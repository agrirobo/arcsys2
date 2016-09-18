#ifndef __IMPL_SEND_POSTURE_H_INCLUDE__
#define __IMPL_SEND_POSTURE_H_INCLUDE__

#include "send_posture.hpp"

class KrsSendPosture : public SendPosture {
public:
  KrsSendPosture(ros::NodeHandle& nh);
  virtual void sendPosture(std::vector<double>& posture);
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  std::vector<int32_t> id_vec;
};

#endif


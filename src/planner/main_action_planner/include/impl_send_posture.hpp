#ifndef __IMPL_SEND_POSTURE_H_INCLUDE__
#define __IMPL_SEND_POSTURE_H_INCLUDE__

#include "send_posture.hpp"

#include "std_srvs/Empty.h"

class KrsSendPosture : public SendPosture {
public:
  KrsSendPosture(ros::NodeHandle& nh);
  virtual void sendPosture(std::vector<double>& posture);
private:
  bool reload();
  bool reload(std_srvs::Empty::Request&  req,
              std_srvs::Empty::Response& res);

  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::ServiceServer reload_srv;
  std::vector<int32_t> id_vec;
};

#endif


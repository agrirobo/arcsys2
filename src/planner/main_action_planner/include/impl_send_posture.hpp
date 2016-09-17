#ifndef __IMPL_SEND_POSTURE_H_INCLUDE__
#define __IMPL_SEND_POSTURE_H_INCLUDE__

#include "send_posture.hpp"

class KrsSendPosture : public SendPosture {
public:
  KrsSendPosture();
  virtual void sendPosture(std::vector<double>& posture);
};

#endif


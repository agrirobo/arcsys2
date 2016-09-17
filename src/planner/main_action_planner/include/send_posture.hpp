#ifndef __SEND_POSTURE_H_INCLUDE__
#define __SEND_POSTURE_H_INCLUDE__

#include <vector>

class SendPosture {
public:
  virtual ~SendPosture(){};
  virtual void sendPosture(std::vector<double>& posture);
}

#endif


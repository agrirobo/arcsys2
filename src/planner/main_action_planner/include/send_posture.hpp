#ifndef __SEND_POSTURE_H_INCLUDE__
#define __SEND_POSTURE_H_INCLUDE__

#include <vector>
#include <string>

class SendPosture {
public:
  virtual ~SendPosture(){};
  virtual void sendPosture(std::vector<double>& posture) = 0;
}

SendPosture* getSendPosture(std::string);

#endif


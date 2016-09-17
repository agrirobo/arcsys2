#ifndef __SEND_POSTURE_H_INCLUDE__
#define __SEND_POSTURE_H_INCLUDE__

#include <vector>
#include <string>
#include <map>

class SendPosture {
public:
  virtual ~SendPosture(){};
  virtual void sendPosture(std::vector<double>& posture) = 0;
}

void getSendPosture(std::map<std::string, SendPosture*> map);

#endif


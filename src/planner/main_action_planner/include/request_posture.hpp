#ifndef __REQUEST_POSTURE_H_INCLUDE__
#define __REQUEST_POSTURE_H_INCLUDE__

#include <vector>
#include <string>
#include <map>

class RequestPosture {
public:
  virtual ~RequestPosure(){};
  virtual void requestPosture(std::vector<double>& posture) = 0;
}

void getRequestPosture(std::map<std::string, RequestPosture*> map);

#endif


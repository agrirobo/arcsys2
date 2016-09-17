#ifndef __REQUEST_POSTURE_H_INCLUDE__
#define __REQUEST_POSTURE_H_INCLUDE__

#include <vector>
#include <string>

class RequestPosture {
public:
  virtual ~RequestPosure(){};
  virtual void requestPosture(std::vector<double>& posture) = 0;
}

RequestPosture* getRequestPosture(std::string name);

#endif


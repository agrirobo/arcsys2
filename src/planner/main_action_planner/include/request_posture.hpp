#ifndef __REQUEST_POSTURE_H_INCLUDE__
#define __REQUEST_POSTURE_H_INCLUDE__

#include <vector>
#include <string>

class RequestPosture {
public:
  virtual ~RequestPosure(){};
  virtual void requestPosture(std::vector<double>& posture) = 0;
}

class RequestPostureFactory {
private:
  RequestPostureFactory();

public:
  static RequestPosture* get(std::string);

private:
  static std::map<std::string, RequestPosture*> reqs;
}

#endif


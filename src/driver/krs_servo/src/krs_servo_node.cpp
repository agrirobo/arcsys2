#include "ros/ros.h"
#include "servo_msgs/KrsServoDegree.h"

#include <string>

#include "krs_servo_driver.hpp"

class KrsServoNode {
public:
  KrsServoNode();
  explicit KrsServoNode(ros::NodeHandle& nh, const char* path);

private:
  void krsServoDegreeCallback(const servo_msgs::KrsServoDegree::ConstPtr& msg);

  ros::Subscriber sub;
  KrsServoDriver krs;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "krs_servo_node");
  ros::NodeHandle nh;

  std::string path;
  nh.param<std::string>("serial_path", path, "/dev/ttyUSB0");

  KrsServoNode ksn(nh, path.c_str());

  ros::spin();
  return 0;
}

KrsServoNode::KrsServoNode()
  : krs()
{}

KrsServoNode::KrsServoNode(ros::NodeHandle& nh, const char* path)
  : krs(path)
{
  sub = nh.subscribe("cmd_krs", 16, &KrsServoNode::krsServoDegreeCallback, this);
}

void KrsServoNode::krsServoDegreeCallback(const servo_msgs::KrsServoDegree::ConstPtr& msg) {
  krs.sendAngle(msg->id, msg->angle);
}

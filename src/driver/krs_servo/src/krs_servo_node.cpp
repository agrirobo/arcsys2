#include "ros/ros.h"
#include "servo_msgs/KrsServoDegree.h"

#include "krs_servo_driver.hpp"

class KrsServoNode {
public:
  KrsServoNode();
  explicit KrsServoNode(ros::NodeHandle& nh);

private:
  void krsServoDegreeCallback(const servo_msgs::KrsServoDegree::ConstPtr& msg);

  ros::Subscriber sub;
  KrsServoDriver krs;
};

KrsServoNode::KrsServoNode()
  : krs()
{}

KrsServoNode::KrsServoNode(ros::NodeHandle& nh)
  : krs()
{
  sub = nh.subscribe("cmd_krs", 16, &KrsServoNode::krsServoDegreeCallback, this);
}

inline void KrsServoNode::krsServoDegreeCallback(const servo_msgs::KrsServoDegree::ConstPtr& msg) {
  krs.sendAngle(msg->id, msg->angle);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "krs_servo_node");
  ros::NodeHandle nh;

  KrsServoNode ksn(nh);

  ros::spin();
  return 0;
}

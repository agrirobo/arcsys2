#include<ros/ros.h>
#include<ics3/ics>
#include<servo_msgs/IdBased.h>

std::string path;

void move(const servo_msgs::IdBased::ConstPtr& msg) {
  static ics::ICS3 ics {path.c_str(), ics::Baudrate::RATE115200()};
  auto degree = ics::Angle::newDegree(msg->angle);
  auto nowpos = ics.move(msg->id, degree);
  // TODO: publish now pos
}
  

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_krs_node");
  ros::NodeHandle n;
  ros::NodeHandle pn {"~"};
  pn.param<std::string>("path", path, "ttyUSB0");
  ros::Subscriber sub = n.subscribe("krs", 100, move);
  ros::spin();
  return 0;
}

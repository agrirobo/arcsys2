#include<ros/ros.h>
#include<ics3/ics>
#include<servo_msgs/IdBased.h>

ics::ICS3* driver;

void move(const servo_msgs::IdBased::ConstPtr& msg) {
  auto degree = ics::Angle::newDegree(msg->angle);
  auto nowpos = driver->move(msg->id, degree);
  // TODO: publish now pos
}
  

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_krs_node");
  ros::NodeHandle n {};
  ros::NodeHandle pn {"~"};
  std::string path {"ttyUSB0"};
  pn.param<std::string>("path", path, path);
  ics::ICS3 ics {path.c_str()};
  driver = &ics;
  ros::Subscriber sub = n.subscribe("krs", 100, move);
  ros::spin();
  return 0;
}

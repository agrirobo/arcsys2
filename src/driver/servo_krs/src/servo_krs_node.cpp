#include<ros/ros.h>
#include<servo_msgs/IdBased.h>
#include<ics3/ics>

ics::ICS3* driver {nullptr};
ros::Publisher pub;

void move(const servo_msgs::IdBased::ConstPtr& msg) {
  servo_msgs::IdBased result;
  try {
    auto now_pos = driver->move(msg->id, ics::Angle::newRadian(msg->angle));
    result.angle = std::move(now_pos);
    result.id = msg->id;
    pub.publish(std::move(result));
  } catch (std::runtime_error e) {
    ROS_INFO("Communicate error: %s", e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_krs_node");
  ros::NodeHandle pn {"~"};
  std::string path {"/dev/ttyUSB0"};
  pn.param<std::string>("path", path, path);
  ROS_INFO("Open servo motor path: %s", path.c_str());
  ics::ICS3 ics {std::move(path)};
  driver = &ics;
  ros::NodeHandle n {};
  ros::Subscriber sub {n.subscribe("cmd_krs", 100, move)};
  pub = n.advertise<servo_msgs::IdBased>("pose_krs", 10);
  ros::spin();
  return 0;
}

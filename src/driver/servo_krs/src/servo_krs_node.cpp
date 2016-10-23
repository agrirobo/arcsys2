#include<ros/ros.h>
#include<ics3/ics>
#include<servo_msgs/IdBased.h>

ics::ICS3* driver {nullptr};
ros::Publisher pub;

void move(const servo_msgs::IdBased::ConstPtr& msg) {
  auto degree = ics::Angle::newDegree(msg->angle);
  try {
    auto nowpos = driver->move(msg->id, degree);
    servo_msgs::IdBased result;
    result.id = msg->id;
    result.angle = nowpos;
    pub.publish(result);
  } catch (std::runtime_error e) {
    ROS_INFO("Communicate error: %s", e.what());
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_krs_node");
  ros::NodeHandle n {};
  ros::NodeHandle pn {"~"};
  std::string path {"/dev/ttyUSB0"};
  pn.param<std::string>("path", path, path);
  try {
    ics::ICS3 ics {path.c_str()};
    driver = &ics;
  } catch (std::runtime_error e) {
    ROS_INFO("Error: Cannot make ICS3 instance [%s]", e.what());
    ROS_INFO("I tried open [%s]", path.c_str());
    return -1;
  }
  ros::Subscriber sub = n.subscribe("cmd_krs", 100, move);
  pub = n.advertise<servo_msgs::IdBased>("pose_krs", 10);
  ros::spin();
  return 0;
}

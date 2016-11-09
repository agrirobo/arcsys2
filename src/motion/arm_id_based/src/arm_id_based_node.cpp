#include"ros/ros.h"
#include"arm_msgs/ArmAnglesDegree.h"
#include"servo_msgs/IdBased.h"

#include<vector>

void armMsgCb(const arm_msgs::ArmAnglesDegree::ConstPtr& msg);

ros::Publisher pub;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "arm_id_based_node");
  ros::NodeHandle pnh("~");
  std::vector<int> id_vec;
  pnh.getParam("id", id_vec);
  if (id_vec.empty()) {
    ROS_ERROR("I need id parameter: id is int vector");
    return 1;
  }
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("arm_roll", 5, armMsgCb);
  pub = nh.advertise<servo_msgs::IdBased>("cmd_krs", 10);
  ros::spin();

  return 0;
}

void armMsgCb(const arm_msgs::ArmAnglesDegree::ConstPtr& msg) {
  
}

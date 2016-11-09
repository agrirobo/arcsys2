#include"ros/ros.h"
#include"arm_msgs/ArmAnglesDegree.h"
#include"servo_msgs/IdBased.h"

#include<vector>

void armMsgCb(const arm_msgs::ArmAnglesDegree::ConstPtr& msg);

ros::Publisher pub;
std::vector<int> id_vec;
std::vector<int>::size_type length; // cache of id_vec length

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "arm_id_based_node");
  ros::NodeHandle pnh("~");
  if (!pnh.getParam("id", id_vec)) {
    ROS_ERROR("I need id parameter: id is int vector");
    return 1;
  }
  length = id_vec.size();
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("arm_roll", 5, armMsgCb);
  pub = nh.advertise<servo_msgs::IdBased>("cmd_krs", 10);
  ros::spin();

  return 0;
}

void armMsgCb(const arm_msgs::ArmAnglesDegree::ConstPtr& msg) {
  if (msg->angles.size() != length) {
    ROS_ERROR("Receive length not equal id vector's: receive [%lu] / id_vec [%lu]", msg->angles.size(), id_vec.size());
    return;
  }
  for (std::size_t i = 0; i < length; i++) {
    servo_msgs::IdBased send;
    send.id = id_vec[i];
    send.angle = msg->angles[i];
    pub.publish(send);
  }
}

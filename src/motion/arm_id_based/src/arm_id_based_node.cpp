#include"ros/ros.h"
#include"arm_msgs/ArmAnglesRadian.h"
#include"servo_msgs/IdBased.h"

#include<vector>
#include<algorithm>

void armMsgCb(const arm_msgs::ArmAnglesRadian::ConstPtr& msg);
void nowPosCb(const servo_msgs::IdBased::ConstPtr& msg);

ros::Publisher move_pub;
ros::Publisher pos_pub;
std::vector<int> id_vec;
std::vector<int>::size_type length; // cache of id_vec length
std::vector<int>::iterator id_begin_it; // cache of id_vec begin
std::vector<int>::iterator id_end_it; // cache of id_vec end
std::vector<double> now_pos;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "arm_id_based_node");
  ros::NodeHandle pnh("~");
  if (!pnh.getParam("id", id_vec)) {
    ROS_ERROR("I need id parameter: id is int vector");
    return 1;
  }
  id_begin_it = id_vec.begin();
  id_end_it = id_vec.end();
  length = id_vec.size();
  now_pos.resize(length);
  ros::NodeHandle nh;
  move_pub = nh.advertise<servo_msgs::IdBased>("cmd_krs", 10);
  pos_pub = nh.advertise<arm_msgs::ArmAnglesRadian>("arm_pos", 3);
  ros::Subscriber move_sub = nh.subscribe("arm_roll", 5, armMsgCb);
  ros::Subscriber pos_sub = nh.subscribe("pose_krs", 5, nowPosCb);
  ros::spin();

  return 0;
}

void armMsgCb(const arm_msgs::ArmAnglesRadian::ConstPtr& msg) {
  if (msg->angles.size() != length) {
    ROS_ERROR("Receive length not equal id vector's: receive [%lu] / id_vec [%lu]", msg->angles.size(), id_vec.size());
    return;
  }
  for (std::size_t i = 0; i < length; i++) {
    servo_msgs::IdBased send;
    send.id = id_vec[i];
    send.angle = msg->angles[i];
    move_pub.publish(send);
  }
}

void nowPosCb(const servo_msgs::IdBased::ConstPtr& msg) {
  std::vector<int>::iterator receive = std::find(id_begin_it, id_end_it, msg->id);
  if (receive == id_end_it) {
    ROS_ERROR("NOT registing id: %d", msg->id);
    return;
  }
  now_pos[receive - id_begin_it] = msg->angle;
  arm_msgs::ArmAnglesRadian send;
  for (std::size_t i = 0; i < length; i++) send.angles.push_back(now_pos[i]);
  pos_pub.publish(send);
}

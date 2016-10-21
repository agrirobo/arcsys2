#include "main_action_planner_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_action_planner_node");
  ros::NodeHandle nh {"~"};
  StateChecker status {nh};

  while (ros::ok()) {
    auto posture_key = status.getPostureKey();
    RequestPosture* rp_p = RequestPostureFactory::get(posture_key[0], nh);
    if (!rp_p) {
      ROS_INFO("Error: get nullptr by RP Factory");
      return -1;
    }

    SendPosture* sp_p = SendPostureFactory::get(posture_key[1], nh);
    if (!sp_p) {
      ROS_INFO("Error: get nullptr by SP Factory");
      return -1;
    }

    std::vector<double> angles;
    rp_p->requestPosture(angles);
    sp_p->sendPosture(angles);

    ros::spinOnce();
  }
  return 0;
}

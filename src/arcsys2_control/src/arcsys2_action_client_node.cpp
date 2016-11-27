#include <string>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>

class Arcsys2ActionClient {
  actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> action_client_;

public:
  Arcsys2ActionClient(const std::string& client_name)
    : action_client_ {client_name, true}
  {
    while (!action_client_.waitForServer(ros::Duration(1.0)))
      ROS_INFO_STREAM("waiting for joint_trajectory_action server");
  }

  void startTrajectory(control_msgs::JointTrajectoryGoal goal) {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    action_client_.sendGoal(goal);
  }

  // control_msgs::JointTrajectoryGoal armExtensionTrajectory() {
  //
  // }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_action_client_node");

  // Arcsys2ActionClient action_client {"follow_joint_trajectory"};

  return 0;
}

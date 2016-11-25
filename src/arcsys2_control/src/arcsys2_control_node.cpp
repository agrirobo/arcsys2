#include <string>

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

class Arcsys2ActionServer {
  ros::NodeHandle node_handle_;
  ros::Publisher publisher;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

  trajectory_msgs::JointTrajectory trajectory_to_goal_;

public:
  Arcsys2ActionServer(const std::string& node_name)
    : action_server_ {node_handle_, node_name, false},
      publisher {node_handle_.advertise<trajectory_msgs::JointTrajectory>("/arcsys2_controller/joint_path_command", 1, this)}
  {
    action_server_.registerGoalCallback(boost::bind(&Arcsys2ActionServer::goalCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&Arcsys2ActionServer::preemptCallback, this));
    action_server_.start();
  }

  void goalCallback() {
    ROS_INFO_STREAM("received: " << (trajectory_to_goal_ = action_server_.acceptNewGoal()->trajectory));
    publisher.publish(trajectory_to_goal_);
  }

  void preemptCallback() {
    ROS_INFO_STREAM("preempted");
    action_server_.setPreempted();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_control_node");

  // Arcsys2ActionServer action_server {ros::this_node::getName()};
  Arcsys2ActionServer action_server {"follow_joint_trajectory"};

  ros::spin();

  return 0;
}

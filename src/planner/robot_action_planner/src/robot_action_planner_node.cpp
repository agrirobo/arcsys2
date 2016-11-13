#include <iostream>

#include <ros/ros.h>
#include <move_group.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MoveGroupPlanner {
  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroup              move_group;
  moveit::planning_interface::MoveGroup::Plan        move_group_plan;
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::DisplayTrajectory display_trajectory;

  ros::Publisher  display_publisher;
  ros::Subscriber subscriber;

public:
  MoveGroupPlanner(const std::string& group)
    : move_group { group },
      publisher  { nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true) },
      subscriber { nh.subscribe<geometry_msgs::Pose>("topic", 10, callback) }
  {
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", move_group.getEndEffectorFrame().c_str());
  }

private:
  void callback(const geometry_msgs::Pose::ConstPtr& msg) {
    move_group.setPoseTarget(msg);

    if (bool success = move_group(move_group_plan)) {
      ROS_INFO("Visualizing plan (pose goal) %s", success ? "" : "FAILED");
      sleep(5.0);
    }

    ROS_INFO("Visualizing plan (again)");
    display_trajectory.trajectory_start = move_group_plan.start_state_;
    display_trajectory.trajectory.push_back(move_group_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    sleep(5.0);

    move_group.move();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_action_planner_node");

  MoveGroupPlanner planner {"arcsys2"};

  return 0;
}

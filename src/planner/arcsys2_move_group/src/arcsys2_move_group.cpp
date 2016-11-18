#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MoveGroupPlanner {
  ros::NodeHandle nh;
  ros::Publisher  pub;
  ros::Subscriber sub;

  moveit::planning_interface::MoveGroup              group;
  moveit::planning_interface::MoveGroup::Plan        plan;
  moveit::planning_interface::PlanningSceneInterface scene;

  geometry_msgs::Pose initial_pose;

  moveit_msgs::DisplayTrajectory dpy;

public:
  MoveGroupPlanner(const std::string& name)
    : pub   { nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true) },
      sub   { nh.subscribe<geometry_msgs::Pose>("sub_topic", 1, &MoveGroupPlanner::callback, this) },
      group { name },
      initial_pose { group.getCurrentPose().pose }
  {
    // ROS_INFO_STREAM("reference frame: " << group.getPlanningFrame());
    // ROS_INFO_STREAM("reference frame: " << group.getEndEffectorLink());
    ROS_INFO_STREAM("initial pose: " << initial_pose);

    group.setPlannerId("RRTConnectkConfigDefault");
  }

  void callback(const geometry_msgs::Pose::ConstPtr& target_pose) {
    // ROS_INFO_STREAM("message received but don't use this for test");

    // geometry_msgs::Pose random_pose = group.getRandomPose().pose;
    // ROS_INFO_STREAM("random_pose:"                                          << std::endl <<
    //                 "  random_pose.position.x: " << random_pose.position.x  << std::endl <<
    //                 "  random_pose.position.y: " << random_pose.position.y  << std::endl <<
    //                 "  random_pose.position.z: " << random_pose.position.z  << std::endl);

    // ROS_INFO_STREAM(group.getCurrentPose());
    // ROS_INFO_STREAM(group.getGoalPositionTolerance());

    group.setPoseTarget(*target_pose, group.getEndEffectorLink());

    if (!group.plan(plan)) ROS_FATAL_STREAM("unable to plan");

    sleep(3.0);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_action_planner_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  MoveGroupPlanner planner {"arcsys2"};

  while (ros::ok()) {
    ros::spin();
    sleep(5.0);
  }

  return 0;
}

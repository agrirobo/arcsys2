#include <string>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MoveGroupPlanner {
  ros::NodeHandle nh;
  ros::Publisher  pub;
  ros::Subscriber sub;

  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::MoveGroup::Plan plan;

  moveit_msgs::DisplayTrajectory dpy;

public:
  MoveGroupPlanner(const std::string& name)
    : pub   { nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true) },
      sub   { nh.subscribe<geometry_msgs::Pose>("sub_topic", 1, &MoveGroupPlanner::callback, this) },
      group { name }
  {
    ROS_INFO_STREAM("Reference frame: " << group.getPlanningFrame());
    ROS_INFO_STREAM("Reference frame: " << group.getEndEffectorLink());
  }

  void callback(const geometry_msgs::Pose::ConstPtr& msg) {
    ROS_INFO_STREAM("Received message "                       << std::endl <<
                    "geometry_msgs::Pose msg"                 << std::endl <<
                    "  msg.position.x: " << (*msg).position.x << std::endl <<
                    "  msg.position.y: " << (*msg).position.y << std::endl <<
                    "  msg.position.z: " << (*msg).position.z                );

    group.setPoseTarget(*msg);

    ROS_INFO_STREAM("Visualizing plan " << (group.plan(plan) ? "" : "failed"));

    dpy.trajectory_start = plan.start_state_;
    dpy.trajectory.push_back(plan.trajectory_);

    pub.publish(dpy);

    sleep(5.0);

//     group.move();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_action_planner_node");

  MoveGroupPlanner planner {"arcsys2"};

  while (ros::ok()) {
    ros::spin();
    sleep(5.0);
  }

  return 0;
}

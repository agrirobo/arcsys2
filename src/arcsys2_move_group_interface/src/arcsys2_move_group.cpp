#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

class MoveGroupPlanner {
  ros::NodeHandle   node;
  ros::Publisher    publisher;
  ros::Subscriber   subscriber;

  const std::string group_name;

  moveit::planning_interface::MoveGroup              move_group;
  moveit::planning_interface::PlanningSceneInterface scene;

public:
  MoveGroupPlanner(const std::string& name)
    : publisher  { node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true) },
      subscriber { node.subscribe<geometry_msgs::Pose>("sub_topic", 1, &MoveGroupPlanner::callback, this) },
      group_name { name },
      move_group { group_name }
  {
    move_group.setPlannerId("RRTConnectkConfigDefault");

    ROS_INFO_STREAM("end effector link: " << move_group.getEndEffectorLink());
    ROS_INFO_STREAM("current pose: "                   << std::endl
                    << getPartitionString('=', 40)     << std::endl
                    << move_group.getCurrentPose().pose
                    << getPartitionString('=', 40)     << std::endl);
  }

  void callback(const geometry_msgs::Pose::ConstPtr& target_pose) {
    ROS_INFO_STREAM("BREAK POINT #1");

    move_group.setPoseTarget(*target_pose, move_group.getEndEffectorLink());

    ROS_INFO_STREAM("BREAK POINT #2");

    ROS_INFO_STREAM("target pose received"             << std::endl
                    << getPartitionString('=', 40)     << std::endl
                    << move_group.getPoseTarget().pose
                    << getPartitionString('=', 40)     << std::endl);

    ROS_INFO_STREAM("BREAK POINT #3");

    moveit::planning_interface::MoveGroup::Plan plan;
    if (move_group.plan(plan)) {
      moveit_msgs::DisplayTrajectory dpy;
      dpy.trajectory_start = plan.start_state_;
      dpy.trajectory.push_back(plan.trajectory_);
      publisher.publish(dpy);

      ROS_INFO_STREAM("BREAK POINT #4");

      sleep(3.0);

      ROS_INFO_STREAM("BREAK POINT #5");

      move_group.move();
    } else {
      ROS_INFO_STREAM("BREAK POINT #6");
    }
  }

private:
  std::string getPartitionString(const char c, const std::size_t size) const {
    std::string tmp(size, c);
    return tmp;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_group_interface_node");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (1) {
    moveit::planning_interface::MoveGroup move_group {"arcsys2"};
    ros::Publisher publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 1.0;
    target_pose.position.y = 1.0;
    target_pose.position.z = 1.5;
    target_pose.orientation.w = 1.0;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroup::Plan plan;
    if (move_group.plan(plan)) {
      moveit_msgs::DisplayTrajectory dpy;
      dpy.trajectory_start = plan.start_state_;
      dpy.trajectory.push_back(plan.trajectory_);
      publisher.publish(dpy);

      sleep(5.0);

      move_group.move();
    }

    sleep(10.0);

    //geometry_msgs::Pose target_pose;
    target_pose.position.x = 1.0;
    target_pose.position.y = -1.0;
    target_pose.position.z = 1.5;
    target_pose.orientation.w = 1.0;

    move_group.setPoseTarget(target_pose);

    //moveit::planning_interface::MoveGroup::Plan plan;
    if (move_group.plan(plan)) {
      moveit_msgs::DisplayTrajectory dpy;
      dpy.trajectory_start = plan.start_state_;
      dpy.trajectory.push_back(plan.trajectory_);
      publisher.publish(dpy);

      sleep(5.0);

      move_group.move();
    }
  }

  ros::spin();

  //MoveGroupPlanner planner {"arcsys2"};

  // ros::waitForShutdown();

  return 0;
}
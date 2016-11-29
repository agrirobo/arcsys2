#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

class TargetStatus {
  ros::Subscriber sub_;

  geometry_msgs::Point point_;

public:
  TargetStatus(ros::NodeHandle node_handle)
    : sub_ {node_handle.subscribe<geometry_msgs::Point>("/tomato_point", 1, &TargetStatus::callback, this)},
      point_ {}
  {
  }

  geometry_msgs::Point getTargetPoint() const { return point_; }

private:
  void callback(const geometry_msgs::PointConstPtr& point) {
    point_ = *point;
    ROS_INFO_STREAM(point_);
  }
};

class MoveGroupInterface {
  ros::Subscriber sub_;

  moveit::planning_interface::MoveGroup move_group_;

public:
  MoveGroupInterface(ros::NodeHandle& node_handle)
    : sub_ {node_handle.subscribe<geometry_msgs::Point>("/tomato_point", 1, &MoveGroupInterface::callback, this)},
      move_group_ {"arcsys2"}
  {
  }

private:
  void callback(const geometry_msgs::PointConstPtr& point)
  {
    geometry_msgs::Pose pose_;
    pose_.position = *point;
    pose_.orientation.w = 1.0;

    move_group_.setPoseTarget(pose_);

    moveit::planning_interface::MoveGroup::Plan plan_;
    if (move_group_.plan(plan_)) ROS_INFO_STREAM("SUCCESS");
    else ROS_INFO_STREAM("FAILED");

    move_group_.execute(plan_);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_move_group_interface_node");

  ros::NodeHandle node_handle {"~"};

  ros::Rate rate {ros::Duration(3.0)};
  ros::AsyncSpinner spinner {1};

  MoveGroupInterface interface {node_handle};

  spinner.start();

  while (ros::ok()) ros::spin();

  spinner.stop();

  return 0;
}

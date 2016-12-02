#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class MoveGroupInterface {
  moveit::planning_interface::MoveGroup move_group_;

  ros::Subscriber sub_;

public:
  MoveGroupInterface(const std::string& group_name, ros::NodeHandle& node_handle)
    : move_group_ {group_name},
      sub_ {node_handle.subscribe<geometry_msgs::PointStamped>("/tomato_point_trusted", 1, &MoveGroupInterface::callback, this)}
  {
  }

private:
  void callback(const geometry_msgs::PointStampedConstPtr& msg)
  {
    geometry_msgs::PoseStamped target_pose;
    target_pose.header = msg->header;
    target_pose.pose.position = msg->point;
    target_pose.pose.orientation.w = 1.0;

    move_group_.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroup::Plan plan;
    if (move_group_.plan(plan)) ROS_INFO_STREAM("SUCCESS");
    else ROS_INFO_STREAM("FAILED");

    move_group_.execute(plan);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_move_group_interface_node");

  ros::NodeHandle node_handle {"~"};
  ros::AsyncSpinner spinner {1};

  MoveGroupInterface interface {"arcsys2", node_handle};

  spinner.start();

  ros::spin();

  spinner.stop();

  return 0;
}

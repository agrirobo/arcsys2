#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

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

class MoveGroupInterfaceTest {
  moveit::planning_interface::MoveGroup move_group_;
  moveit::planning_interface::MoveGroup::Plan plan_;

  geometry_msgs::Pose target_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

public:
  MoveGroupInterfaceTest(const std::string group_name)
    : move_group_ {group_name},
      plan_ {},
      buffer_ {},
      listener_ {buffer_}
  {
  }

  bool getTomatoPoint()
  {
    try {
      // geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform(move_group_.getPlanningFrame(), "tomato", ros::Time(0), ros::Duration(5.0))};
      geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform("rail", "tomato", ros::Time(0), ros::Duration(5.0))};

      target_.position.x = transform_stamped_.transform.translation.x;
      target_.position.y = transform_stamped_.transform.translation.y;
      target_.position.z = transform_stamped_.transform.translation.z;
      target_.orientation.w = 1.0;

    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      return false;
    }

    return true;
  }

  bool setPoseToApproach()
  {
    target_.position.x -= 0.1;
    return move_group_.setPoseTarget(target_);
  }

  bool setPoseToInsert()
  {
    target_.position.x += 0.1;
    return move_group_.setPoseTarget(target_);
  }

  bool setPoseToCut()
  {
    target_.orientation.w -= 0.1;
    return move_group_.setPoseTarget(target_);
  }

  bool plan() { return move_group_.plan(plan_); }

  bool execute() { return move_group_.execute(plan_); }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_move_group_interface_node");

  ros::NodeHandle node_handle {"~"};
  ros::AsyncSpinner spinner {1};

  MoveGroupInterfaceTest interface {"arcsys2"};

  spinner.start();

  ros::spin();

  spinner.stop();

  return 0;
}

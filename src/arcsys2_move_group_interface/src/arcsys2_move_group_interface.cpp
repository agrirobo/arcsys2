#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

class MoveGroupInterfaceOld {
  moveit::planning_interface::MoveGroup move_group_;
  moveit::planning_interface::MoveGroup::Plan motion_plan_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::Pose target_pose_;

public:
  MoveGroupInterfaceOld(const std::string& group_name, const double& joint_tolerance = 0.1)
    : move_group_ {group_name},
      motion_plan_ {},
      buffer_ {},
      listener_ {buffer_}
  {
    move_group_.allowReplanning(true);
    move_group_.setGoalJointTolerance(joint_tolerance);
  }

  bool getTomatoPoint()
  {
    try {
      geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform("rail", "tomato", ros::Time(0), ros::Duration(5.0))};

      target_pose_.position.x = transform_stamped_.transform.translation.x;
      target_pose_.position.y = transform_stamped_.transform.translation.y;
      target_pose_.position.z = transform_stamped_.transform.translation.z;
      target_pose_.orientation.w = 1.0;

    } catch (const tf2::TransformException& ex) {
      ROS_WARN_STREAM(ex.what());
      return false;
    }

    return true;
  }

  bool setPoseToApproach(const double& effector_length)
  {
    target_pose_.position.x -= effector_length;
    return move_group_.setPoseTarget(target_pose_);
  }

  bool setPoseToInsert(const double& effector_length)
  {
    target_pose_.position.x += effector_length;
    return move_group_.setPoseTarget(target_pose_);
  }

  bool setPoseToCut(const double& radian)
  {
    // tf::createQuaternionMsgFromRollPitchYaw(1.0, 0.0, 0.0);

    target_pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(1, 0, 0);
    return move_group_.setPoseTarget(target_pose_);
  }

  bool setPoseToWait()
  {
    target_pose_.position.x = 1.0;
    // target_pose_.position.y =
    target_pose_.position.z = 1.5;
    target_pose_.orientation.w = 1.0;
  }

  bool move()
  {
    if (!move_group_.plan(motion_plan_)) return false;
    move_group_.execute(motion_plan_);
    return true;
  }
};

class MoveGroupInterface {
  moveit::planning_interface::MoveGroup move_group_;
  moveit::planning_interface::MoveGroup::Plan motion_plan_;

  geometry_msgs::Point tomapo_;
  std::vector<geometry_msgs::Pose> waypoints_;

  static constexpr double eef_length {0.5}; // TODO
  static constexpr double eef_step {0.01};

public:
  MoveGroupInterface(const std::string& group_name, const double& joint_tolerance)
    : move_group_ {group_name},
      motion_plan_ {}
  {
    move_group_.allowReplanning(true);
    move_group_.setGoalJointTolerance(joint_tolerance);
  }

  bool queryTargetExistence()
  {
    try {
      geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform("rail", "tomato", ros::Time(0), ros::Duration(5.0))};
      tomapo_.position.x = transform_stamped_.transform.translation.x;
      tomapo_.position.y = transform_stamped_.transform.translation.y;
      tomapo_.position.z = transform_stamped_.transform.translation.z;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_INFO_STREAM(ex.what());
      return false;
    }
  }

private:
  void setCartesianPaths(const geometry_msgs::Pose& from)
  {
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arcsys2_move_group_interface_node");

  ros::NodeHandle node_handle {"~"};
  ros::Rate rate {ros::Duration(1.0)};

  MoveGroupInterface interface {"arcsys2", node_handle.param("joint_tolerance", 0.1)};

  ros::AsyncSpinner spinner {1};
  spinner.start();

  while (ros::ok()) {
    rate.sleep();
  }

  spinner.stop();

  return 0;
}

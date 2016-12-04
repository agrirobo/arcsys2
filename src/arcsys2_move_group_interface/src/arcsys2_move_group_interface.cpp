#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

class MoveGroupInterface {
  moveit::planning_interface::MoveGroup move_group_;
  moveit::planning_interface::MoveGroup::Plan motion_plan_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::Pose target_pose_;

public:
  MoveGroupInterface(const std::string& group_name,
                     const double& joint_tolerance = 0.1,
                     const double& position_tolerance = 0.1,
                     const double& orientation_tolerance = 0.1)
    : move_group_ {group_name},
      motion_plan_ {},
      buffer_ {},
      listener_ {buffer_}
  {
    move_group_.setGoalJointTolerance(joint_tolerance);
    move_group_.setGoalPositionTolerance(position_tolerance);
    move_group_.setGoalOrientationTolerance(orientation_tolerance);
  }

  bool getTomatoPoint()
  {
    try {
      // geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform(move_group_.getPlanningFrame(), "tomato", ros::Time(0), ros::Duration(5.0))};
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

  bool setPoseToCut(const double& effector_length)
  {
    target_pose_.orientation.w -= effector_length;
    return move_group_.setPoseTarget(target_pose_);
  }

  bool move()
  {
    if (!move_group_.plan(motion_plan_)) return false;
    move_group_.execute(motion_plan_);
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_move_group_interface_node");
  ros::NodeHandle node_handle {"~"};

  MoveGroupInterface interface {"arcsys2",
                                node_handle.param("joint_tolerance", 0.1),
                                node_handle.param("position_tolerance", 0.1),
                                node_handle.param("orientation_tolerance", 0.1)};

  while (ros::ok()) {
    if (interface.getTomatoPoint()) {
      if (interface.setPoseToApproach(0.1)) interface.move();
      if (interface.setPoseToInsert(0.1)) interface.move();
      if (interface.setPoseToCut(0.1)) interface.move();
    }
    ros::spin();
  }

  return 0;
}

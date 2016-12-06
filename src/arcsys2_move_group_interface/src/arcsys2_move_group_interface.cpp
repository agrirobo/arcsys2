#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <moveit_msgs/RobotTrajectory.h>

class MoveGroupInterface {
  moveit::planning_interface::MoveGroup move_group_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::Pose tomapo_;
  std::vector<geometry_msgs::Pose> waypoints_;

  static constexpr double eef_length {0.3}; // TODO
  static constexpr double eef_step {0.10};

public:
  MoveGroupInterface(const std::string& group_name, const double& joint_tolerance)
    : move_group_ {group_name},
      buffer_ {},
      listener_ {buffer_},
      tomapo_ {},
      waypoints_ {}
  {
    move_group_.allowReplanning(true);
    move_group_.setGoalJointTolerance(joint_tolerance);
    move_group_.setPlanningTime(5.0);
  }

  bool queryTargetExistence()
  {
    try {
      geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform("rail", "tomato", ros::Time(0), ros::Duration(5.0))};
      tomapo_.position.x = transform_stamped_.transform.translation.x;
      tomapo_.position.y = transform_stamped_.transform.translation.y;
      tomapo_.position.z = transform_stamped_.transform.translation.z;
      tomapo_.orientation.x = 0;
      tomapo_.orientation.y = 0;
      tomapo_.orientation.z = 0;
      tomapo_.orientation.w = 1.0;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_INFO_STREAM(ex.what());
      return false;
    }
  }

  bool startSequence()
  {
    moveit_msgs::RobotTrajectory trajectory_msgs_ {getCartesianPaths()};

    robot_trajectory::RobotTrajectory robot_trajectory_ {move_group_.getCurrentState()->getRobotModel(), move_group_.getName()};
    robot_trajectory_.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory_msgs_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(robot_trajectory_);

    robot_trajectory_.getRobotTrajectoryMsg(trajectory_msgs_);

    moveit::planning_interface::MoveGroup::Plan motion_plan_;
    motion_plan_.trajectory_ = trajectory_msgs_;

    return move_group_.execute(motion_plan_);
  }

private:
  moveit_msgs::RobotTrajectory getCartesianPaths()
  {
    waypoints_.push_back(linear(tomapo_, -eef_length));
    waypoints_.push_back(tomapo_);
    waypoints_.push_back(roll(tomapo_, 1.0));
    waypoints_.push_back(linear(tomapo_, -eef_length));

    moveit_msgs::RobotTrajectory robot_trajectory_;
    move_group_.computeCartesianPath(waypoints_, eef_step, 0.0, robot_trajectory_);

    return robot_trajectory_;
  }

  geometry_msgs::Pose linear(const geometry_msgs::Pose& pose, const double& distance)
  {
    geometry_msgs::Pose pose_ {pose};
    pose_.position.x += distance;
    return pose_;
  }

  geometry_msgs::Pose roll(const geometry_msgs::Pose& pose, const double& radian)
  {
    geometry_msgs::Pose pose_ {pose};
    pose_.orientation = tf::createQuaternionMsgFromRollPitchYaw(radian, 0, 0);
    return pose_;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arcsys2_move_group_interface_node");

  ros::NodeHandle node_handle {"~"};
  // ros::Rate rate {ros::Duration(10.0)};

  MoveGroupInterface interface {"arcsys2", node_handle.param("joint_tolerance", 0.1)};

  ros::AsyncSpinner spinner {1};
  spinner.start();

  while (ros::ok()) {
    if (interface.queryTargetExistence()) interface.startSequence();
    // rate.sleep();
  }

  spinner.stop();

  return 0;
}

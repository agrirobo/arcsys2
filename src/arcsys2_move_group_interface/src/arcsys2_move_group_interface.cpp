#include <string>
#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit_msgs/RobotTrajectory.h>

class MoveGroupInterface {
  moveit::planning_interface::MoveGroup move_group_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::Pose tomapo_;
  std::vector<geometry_msgs::Pose> waypoints_;

  double eef_length_;
  double shift_margin_;
  bool reverse_coordinate_;

  moveit::core::VariableBounds rail_bounds_;
  double sign_;

public:
  MoveGroupInterface(const std::string& group_name, const ros::NodeHandle& node_handle)
    : move_group_ {group_name},
      buffer_ {},
      listener_ {buffer_},
      tomapo_ {},
      waypoints_ {},
      rail_bounds_ {move_group_.getRobotModel()->getJointModel("rail_to_base_joint")->getVariableBounds()[0]},
      sign_ {1.0}
  {
    node_handle.getParam("effector_length", eef_length_);
    node_handle.getParam("shift_margin", shift_margin_);
    node_handle.getParam("reverse_coordinate", reverse_coordinate_);

    double joint_tolerance;
    node_handle.getParam("joint_tolerance", joint_tolerance);
    move_group_.setGoalJointTolerance(joint_tolerance);

    move_group_.allowReplanning(true);
    move_group_.setPlanningTime(5.0);
  }

  bool queryTargetExistence()
  {
    try {
      geometry_msgs::TransformStamped transform_stamped_ {buffer_.lookupTransform("rail", "tomato", ros::Time(0), ros::Duration(1.0))};
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
    waypoints_.clear();

    geometry_msgs::Pose pose1 {tomapo_};
    pose1.position.x -= eef_length_;
    move_group_.setPoseTarget(pose1);

    moveit::planning_interface::MoveGroup::Plan plan; // NEED?
    move_group_.plan(plan);
    if (!move_group_.execute(plan)) return false;

    geometry_msgs::Pose pose2 {tomapo_};
    waypoints_.push_back(pose2);

    geometry_msgs::Pose pose3 {tomapo_};
    pose3.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.0, 0, 0); // TODO
    waypoints_.push_back(pose3);

    geometry_msgs::Pose pose4 {tomapo_};
    pose4.position.x -= eef_length_;
    waypoints_.push_back(pose4);

    return move_group_.execute(planCartesianPath(waypoints_));
  }

  bool shift()
  {
    std::vector<double> joint_values {move_group_.getCurrentJointValues()};

    joint_values[1] =  0;
    joint_values[2] = -0.7854;
    joint_values[3] =  1.5707;
    joint_values[4] = -0.7854;
    joint_values[5] =  0;

    move_group_.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroup::Plan plan;
    move_group_.plan(plan);

    move_group_.execute(plan);

    // if (joint_values[0] > (rail_bounds_.max_position_ - shift_margin_)) sign_ = -1.0;
    // else if (joint_values[0] < (rail_bounds_.min_position_ + shift_margin_)) sign_ = 1.0;

    updateShiftSign(joint_values[0]);
    joint_values[0] += sign_ * 0.5;

    move_group_.setJointValueTarget(joint_values);
    move_group_.plan(plan);

    return move_group_.execute(plan);
  }

private:
  moveit::planning_interface::MoveGroup::Plan planCartesianPath(std::vector<geometry_msgs::Pose>& waypoints)
  {
    moveit_msgs::RobotTrajectory msg;
    move_group_.computeCartesianPath(waypoints, 0.10, 0.0, msg);

    robot_trajectory::RobotTrajectory trajectory {move_group_.getCurrentState()->getRobotModel(), move_group_.getName()};
    trajectory.setRobotTrajectoryMsg(*move_group_.getCurrentState(), msg);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(trajectory);

    moveit::planning_interface::MoveGroup::Plan plan;
    plan.trajectory_ = msg;

    return plan;
  }

  void updateShiftSign(const double& joint_value)
  {
    if (rail_bounds_.max_position_ - shift_margin_ < joint_value)
      reverse_coordinate_ ? sign_ = +1.0 : sign_ = -1.0;

    if (joint_value < 0 + shift_margin_)
      reverse_coordinate_ ? sign_ = -1.0 : sign_ = +1.0;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arcsys2_move_group_interface_node");
  ros::NodeHandle node_handle {"~"};

  ros::AsyncSpinner spinner {1};
  spinner.start();

  MoveGroupInterface interface {"arcsys2", node_handle};

  while (ros::ok()) {
    while (!interface.queryTargetExistence()) interface.shift();
    interface.startSequence();
  }

  spinner.stop();

  return 0;
}

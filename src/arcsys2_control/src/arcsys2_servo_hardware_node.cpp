#include <string>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

class ArcsysServoHW : public hardware_interface::RobotHW {
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  double cmd_, pos_, vel_, eff_;

public:
  ArcsysServoHW(const std::string& joint_name) {
    hardware_interface::JointStateHandle joint_state_handle_ {joint_name, &pos_, &vel_, &eff_};
    joint_state_interface_.registerHandle(joint_state_handle_);

    registerInterface(&joint_state_interface_);

    hardware_interface::JointHandle joint_handle_ {joint_state_interface_.getHandle(joint_name), &cmd_};
    position_joint_interface_.registerHandle(joint_handle_);

    registerInterface(&position_joint_interface_);
  };

  void read() {
    ROS_INFO_STREAM("command for joints: " << cmd_);
  };

  void write() {
    ROS_INFO_STREAM("updated pos_: " << (pos_ += cmd_));
  };

  inline ros::Time getTime() const { return ros::Time::now(); }
  inline ros::Duration getPeriod() const { return ros::Duration(0.1); }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "arcsys2_servo_hardware_node");
  ros::NodeHandle pnh {"~"};

  ArcsysServoHW robot_hw {"shaft_to_arm0_joint"};
  controller_manager::ControllerManager controller_manager {&robot_hw, pnh};

  ros::Rate rate {1.0 / robot_hw.getPeriod().toSec()};

  ros::AsyncSpinner spinner {1};
  spinner.start();

  while (ros::ok()) {
    robot_hw.read();
    controller_manager.update(robot_hw.getTime(), robot_hw.getPeriod());
    robot_hw.write();
    rate.sleep();
  }

  spinner.stop();

  return 0;
}

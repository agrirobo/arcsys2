#include <string>
#include <utility>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h> // for PositionJointInterface
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Arcsys2HW
  : public hardware_interface::RobotHW
{
public:
  Arcsys2HW();
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  static constexpr std::size_t JOINT_COUNT {6};
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  double pos_[JOINT_COUNT];
  double vel_[JOINT_COUNT];
  double eff_[JOINT_COUNT];
  double cmd_[JOINT_COUNT];
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arcsys2_control_node");

  Arcsys2HW robot {};
  controller_manager::ControllerManager cm {&robot};

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner {1};

  spinner.start();
  while(ros::ok())
  {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}

inline Arcsys2HW::Arcsys2HW()
: joint_state_interface_ {},
  joint_position_interface_ {},
  joint_velocity_interface_ {},
  cmd_ {},
  pos_ {},
  vel_ {},
  eff_ {}
{
  // [input] connect and register the joint state interface
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"rail_to_shaft_joint", &pos_[0], &vel_[0], &eff_[0]});
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"shaft_to_arm0_joint", &pos_[1], &vel_[1], &eff_[1]});
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"arm0_to_arm1_joint", &pos_[2], &vel_[2], &eff_[2]});
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"arm1_to_arm2_joint", &pos_[3], &vel_[3], &eff_[3]});
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"arm2_to_effector_base_joint", &pos_[4], &vel_[4], &eff_[4]});
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {"effector_base_to_effector_end_joint", &pos_[5], &vel_[5], &eff_[5]});

  registerInterface(&joint_state_interface_);

  // [output] connect and register the joint position interface
  joint_velocity_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("rail_to_shaft_joint"), &cmd_[0]});
  joint_velocity_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("shaft_to_arm0_joint"), &cmd_[1]});
  joint_position_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("arm0_to_arm1_joint"), &cmd_[2]});
  joint_position_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("arm1_to_arm2_joint"), &cmd_[3]});
  joint_position_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("arm2_to_effector_base_joint"), &cmd_[4]});
  joint_position_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle("effector_base_to_effector_end_joint"), &cmd_[5]});

  registerInterface(&joint_velocity_interface_);
  registerInterface(&joint_position_interface_);
}

inline void Arcsys2HW::read()
{
  for (std::size_t i {0}; i < 2; i++)
    vel_[i] = cmd_[i];
  for (std::size_t i {2}; i < 6; i++)
    pos_[i] = cmd_[i];
}

inline void Arcsys2HW::write()
{
}

inline ros::Time Arcsys2HW::getTime() const
{
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const
{
  return ros::Duration(0.01);
}

#include <stdexcept>
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
  Arcsys2HW(const std::string&);
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface joint_position_interface_;
  double pos_;
  double vel_;
  double eff_;
  double cmd_;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arcsys2_control_node");

  std::string joint_name {};
  ros::param::get("~joint_name", joint_name);
  Arcsys2HW robot {std::move(joint_name)};
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

inline Arcsys2HW::Arcsys2HW(const std::string& joint_name)
: joint_state_interface_ {},
  joint_position_interface_ {},
  cmd_ {},
  pos_ {},
  vel_ {},
  eff_ {}
{
  if (joint_name.empty()) throw std::invalid_argument {"Not allow empty joint name"};
  // [input] connect and register the joint state interface
  joint_state_interface_.registerHandle(hardware_interface::JointStateHandle {joint_name, &pos_, &vel_, &eff_});
  registerInterface(&joint_state_interface_);

  // [output] connect and register the joint position interface
  joint_position_interface_.registerHandle(hardware_interface::JointHandle {joint_state_interface_.getHandle(joint_name), &cmd_});
  registerInterface(&joint_position_interface_);
}

inline void Arcsys2HW::read()
{
}

inline void Arcsys2HW::write()
{
  pos_ = cmd_;
}

inline ros::Time Arcsys2HW::getTime() const
{
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const
{
  return ros::Duration(0.01);
}

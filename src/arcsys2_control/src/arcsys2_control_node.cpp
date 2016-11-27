#include <string>
#include <utility>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class Arcsys2HW : public hardware_interface::RobotHW {
public:
  Arcsys2HW();
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  static constexpr std::size_t JOINT_COUNT {6};
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface joint_pos_interface;
  double cmd[JOINT_COUNT];
  double pos[JOINT_COUNT];
  double vel[JOINT_COUNT];
  double eff[JOINT_COUNT];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "arcsys2_control_node");

  Arcsys2HW robot {};
  controller_manager::ControllerManager cm {&robot};

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner {1};
  spinner.start();

  while(ros::ok()) {
    robot.read();
    cm.update(robot.getTime(), robot.getPeriod());
    robot.write();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}

inline Arcsys2HW::Arcsys2HW()
: joint_state_interface {},
  joint_pos_interface {},
  cmd {},
  pos {},
  vel {},
  eff {}
{
  // [input] connect and register the joint state interface
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"rail_to_shaft_joint", &pos[0], &vel[0], &eff[0]});
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"shaft_to_arm0_joint", &pos[1], &vel[1], &eff[1]});
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"arm0_to_arm1_joint", &pos[2], &vel[2], &eff[2]});
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"arm1_to_arm2_joint", &pos[3], &vel[3], &eff[3]});
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"arm2_to_effector_base_joint", &pos[4], &vel[4], &eff[4]});
  joint_state_interface.registerHandle(hardware_interface::JointStateHandle {"effector_base_to_effector_end_joint", &pos[5], &vel[5], &eff[5]});

  registerInterface(&joint_state_interface);

  // [output] connect and register the joint position interface
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("rail_to_shaft_joint"), &cmd[0]});
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("shaft_to_arm0_joint"), &cmd[1]});
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("arm0_to_arm1_joint"), &cmd[2]});
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("arm1_to_arm2_joint"), &cmd[3]});
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("arm2_to_effector_base_joint"), &cmd[4]});
  joint_pos_interface.registerHandle(hardware_interface::JointHandle {joint_state_interface.getHandle("effector_base_to_effector_end_joint"), &cmd[5]});

  registerInterface(&joint_pos_interface);
}

inline void Arcsys2HW::read() {
  for (std::size_t i {0}; i < 6; i++) ROS_INFO_STREAM("cmd[" << i << "]: "  << cmd[i]);
}

inline void Arcsys2HW::write() {
  for (std::size_t i {0}; i < 6; i++) pos[i] = cmd[i];
}

inline ros::Time Arcsys2HW::getTime() const {
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const {
  return ros::Duration(0.01);
}

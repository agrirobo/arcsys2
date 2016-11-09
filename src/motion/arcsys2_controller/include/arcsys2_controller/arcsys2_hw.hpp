#include"ros/ros.h"
#include"hardware_interface/joint_command_interface.h"
#include"hardware_interface/joint_state_interface.h"
#include"hardware_interface/robot_hw.h"

class Arcsys2HW : public hardware_interface::RobotHW {
public:
  Arcsys2HW();
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  hardware_interface::JointStateInterface jntStateInterface;
  hardware_interface::PositionJointInterface jntPosInterface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

inline Arcsys2HW::Arcsys2HW() {
  // [input] connect and register the joint state interface
  hardware_interface::JointStateHandle stateHandleA("A", &pos[0], &vel[0], &eff[0]);
  jntStateInterface.registerHandle(stateHandleA);

  hardware_interface::JointStateHandle stateHandleB("B", &pos[1], &vel[1], &eff[1]);
  jntStateInterface.registerHandle(stateHandleB);

  registerInterface(&jntStateInterface);

  // [output] connect and register the joint position interface
  hardware_interface::JointHandle posHandleA(jntStateInterface.getHandle("A"), &cmd[0]);
  jntPosInterface.registerHandle(posHandleA);

  hardware_interface::JointHandle posHandleB(jntStateInterface.getHandle("B"), &cmd[1]);
  jntPosInterface.registerHandle(posHandleB);

  registerInterface(&jntPosInterface);
}

inline void Arcsys2HW::read() {
  // TODO: read encoder
}

inline void Arcsys2HW::write() {
  // TODO: write KRS servo and base motor
  // TODO: receive KRS angles
}

inline ros::Time Arcsys2HW::getTime() const {
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const {
  return ros::Duration(0.01);
}

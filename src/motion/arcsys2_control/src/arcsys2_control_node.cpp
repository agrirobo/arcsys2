#include"ros/ros.h"
#include"controller_manager/controller_manager.h"
#include"hardware_interface/joint_command_interface.h"
#include"hardware_interface/joint_state_interface.h"
#include"hardware_interface/robot_hw.h"

#include"ics3/ics"

#include<string>

class Arcsys2HW : public hardware_interface::RobotHW {
public:
  Arcsys2HW(const std::string&);
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  // for real move
  ics::ICS3 krs_driver;
  // for RobotHW
  hardware_interface::JointStateInterface jntStateInterface;
  hardware_interface::PositionJointInterface jntPosInterface;
  double arm_cmd[2];
  double arm_pos[2];
  double arm_vel[2];
  double arm_eff[2];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "arcsys2_control_node");

  ros::NodeHandle pnh {"~"};
  std::string krs_path {"/dev/ttyUSB0"};
  pnh.param<std::string>("krs_path", krs_path, krs_path);
  Arcsys2HW robot {krs_path};
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

inline Arcsys2HW::Arcsys2HW(const std::string& krs_path)
: krs_driver {krs_path},
  jntStateInterface {},
  jntPosInterface {}
{
  // [input] connect and register the joint state interface
  hardware_interface::JointStateHandle stateHandle0to1 {"arm0->arm1", &arm_pos[0], &arm_vel[0], &arm_eff[0]};
  jntStateInterface.registerHandle(stateHandle0to1);

  hardware_interface::JointStateHandle stateHandle1to2 {"arm1->arm2", &arm_pos[1], &arm_vel[1], &arm_eff[1]};
  jntStateInterface.registerHandle(stateHandle1to2);

  registerInterface(&jntStateInterface);

  // [output] connect and register the joint position interface
  hardware_interface::JointHandle posHandle0to1 {jntStateInterface.getHandle("arm0->arm1"), &arm_cmd[0]};
  jntPosInterface.registerHandle(posHandle0to1);

  hardware_interface::JointHandle posHandle1to2 {jntStateInterface.getHandle("arm1->arm2"), &arm_cmd[1]};
  jntPosInterface.registerHandle(posHandle1to2);

  registerInterface(&jntPosInterface);
}

inline void Arcsys2HW::read() {
  // TODO: read encoder
}

inline void Arcsys2HW::write() {
  // TODO: write krs servo moveing
  // TODO: write base motor
  // TODO: receive KRS angles
}

inline ros::Time Arcsys2HW::getTime() const {
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const {
  return ros::Duration(0.01);
}

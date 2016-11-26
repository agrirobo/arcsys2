#include"ros/ros.h"
#include"controller_manager/controller_manager.h"
#include"hardware_interface/joint_command_interface.h"
#include"hardware_interface/joint_state_interface.h"
#include"hardware_interface/robot_hw.h"


#include<string>
#include<utility>

class Arcsys2HW : public hardware_interface::RobotHW {
public:
  static constexpr std::size_t krsCount {4};
  using IdContainer = std::array<int, krsCount>;
  Arcsys2HW(std::string&&, IdContainer&&);
  void read();
  void write();
  ros::Time getTime() const;
  ros::Duration getPeriod() const;
private:
  // for real move
  IdContainer krs_ids;
  // for RobotHW
  hardware_interface::JointStateInterface jntStateInterface;
  hardware_interface::PositionJointInterface jntPosInterface;
  double krs_cmd[krsCount];
  double krs_pos[krsCount];
  double krs_vel[krsCount];
  double krs_eff[krsCount];
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "arcsys2_control_node");

  ros::NodeHandle pnh {"~"};
  std::vector<int> armJointIds {};
  std::string krs_path {"/dev/ttyUSB0"};
  pnh.param<std::string>("krs_path", krs_path, krs_path);
  Arcsys2HW robot {std::move(krs_path),
                   Arcsys2HW::IdContainer {0, 1, 2, 3}};
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

inline Arcsys2HW::Arcsys2HW(std::string&& krs_path, IdContainer&& krs_ids)
: 
  krs_ids(std::move(krs_ids)), // call copy (or move) constructor
  jntStateInterface {},
  jntPosInterface {},
  krs_cmd {},
  krs_pos {},
  krs_vel {},
  krs_eff {}
{
  // [input] connect and register the joint state interface
  hardware_interface::JointStateHandle stateHandle0to1 {"arm0_to_arm1", &krs_pos[0], &krs_vel[0], &krs_eff[0]};
  jntStateInterface.registerHandle(stateHandle0to1);

  hardware_interface::JointStateHandle stateHandle1to2 {"arm1_to_arm2", &krs_pos[1], &krs_vel[1], &krs_eff[1]};
  jntStateInterface.registerHandle(stateHandle1to2);

  registerInterface(&jntStateInterface);

  // [output] connect and register the joint position interface
  hardware_interface::JointHandle posHandle0to1 {jntStateInterface.getHandle("arm0_to_arm1"), &krs_cmd[0]};
  jntPosInterface.registerHandle(posHandle0to1);

  hardware_interface::JointHandle posHandle1to2 {jntStateInterface.getHandle("arm1_to_arm2"), &krs_cmd[1]};
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

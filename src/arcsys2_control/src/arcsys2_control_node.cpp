#include <array>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ics3/ics>

class JointControlInterface
{
public:
  virtual void fetch() = 0;
  virtual void move() = 0;
};

struct JointDatas
{
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
};

template<class JntCmdInterface>
void registerJoint(
    JointDatas&,
    hardware_interface::JointStateInterface&,
    JntCmdInterface&);

template<class JntCmdInterface>
struct JointControlBuildData
{
  std::string joint_name_;
  hardware_interface::JointStateInterface& jnt_stat_;
  JntCmdInterface& jnt_cmd_;
};

class ICSControl
  : public JointControlInterface
{
public:
  ICSControl(
      const JointControlBuildData<hardware_interface::PositionJointInterface>&,
      const std::string&,
      const ics::ID&);
  void fetch() override;
  void move() override;
private:
  JointDatas data;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arcsys2_control_node");
//
//  Arcsys2HW robot {};
//  controller_manager::ControllerManager cm {&robot};
//
//  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner {1};

  spinner.start();
  while(ros::ok())
  {
//    robot.read();
//    cm.update(robot.getTime(), robot.getPeriod());
//    robot.write();
//    rate.sleep();
  }
  spinner.stop();

  return 0;
}

template<class JntCmdInterface>
void registerJoint(
    JointDatas& joint,
    hardware_interface::JointStateInterface& jnt_state,
    JntCmdInterface& jnt_cmd)
{
  jnt_state.registerHandle(hardware_interface::JointStateHandle {joint.name_, joint.pos_, joint.vel_, joint.eff_});
  jnt_cmd.registerHandle(hardware_interface::JointHandle {jnt_state.getHandle(joint.name_), joint.cmd_});
}

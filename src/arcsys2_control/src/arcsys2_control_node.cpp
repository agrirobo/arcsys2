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

struct JointData
{
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
};

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
  using JntCmdType = hardware_interface::PositionJointInterface;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  ICSControl(
      BuildDataType&,
      const std::string&,
      const ics::ID&);
  void fetch() override;
  void move() override;
private:
  JointData data_;
};

template<class JntCmdInterface>
void registerJoint(
    JointData&,
    hardware_interface::JointStateInterface&,
    JntCmdInterface&);

template<class JntCmdInterface>
void registerJoint(
    JointData&,
    JointControlBuildData<JntCmdInterface>&);

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
inline void registerJoint(
    JointData& joint,
    hardware_interface::JointStateInterface& jnt_stat,
    JntCmdInterface& jnt_cmd)
{
  jnt_stat.registerHandle(hardware_interface::JointStateHandle {joint.name_, &joint.pos_, &joint.vel_, &joint.eff_});
  jnt_cmd.registerHandle(hardware_interface::JointHandle {jnt_stat.getHandle(joint.name_), &joint.cmd_});
}

template<class JntCmdInterface>
inline void registerJoint(
    JointData& joint,
    JointControlBuildData<JntCmdInterface>& build_data)
{
  registerJoint(joint, build_data.jnt_stat_, build_data.jnt_cmd_);
}

ICSControl::ICSControl(
      BuildDataType& build_data,
      const std::string& device_path,
      const ics::ID& id)
  : data_ {build_data.joint_name_}
{
  registerJoint(data_, build_data);
}

void ICSControl::fetch()
{
}

void ICSControl::move()
{
}

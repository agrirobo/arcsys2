#include <array>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ics3/ics>

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

class JointControlInterface
{
public:
  virtual void fetch() = 0;
  virtual void move() = 0;
};

class ICSControl
  : public JointControlInterface
{
public:
  using JntCmdType = hardware_interface::PositionJointInterface;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  ICSControl(BuildDataType&, const std::string&, const ics::ID&);
  void fetch() override;
  void move() override;
private:
  JointData data_;
};

class DammyControl
  : public JointControlInterface
{
public:
  using JntCmdType = hardware_interface::PositionJointInterface;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  DammyControl(BuildDataType&);
  void fetch() override;
  void move() override;
private:
  JointData data_;
};

class Arcsys2HW
  : public hardware_interface::RobotHW
{
public:
  static constexpr std::size_t JOINT_COUNT {6};
  using JointControlContainer = std::array<JointControlInterface*, JOINT_COUNT>;
  Arcsys2HW(hardware_interface::JointStateInterface*);
  void registerControl(JointControlInterface*);
private:
  JointControlContainer controls;
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
  hardware_interface::JointStateInterface joint_state_interface {};
  hardware_interface::PositionJointInterface position_joint_interface {};

  DammyControl::BuildDataType shaft_builder {"rail_to_shaft_joint", joint_state_interface, position_joint_interface};
  DammyControl shaft_control {shaft_builder};
  DammyControl::BuildDataType arm0_builder {"shaft_to_arm0_joint", joint_state_interface, position_joint_interface};
  DammyControl arm0_control {arm0_builder};
  DammyControl::BuildDataType arm1_builder {"arm0_to_arm1_joint", joint_state_interface, position_joint_interface};
  DammyControl arm1_control {arm1_builder};
  DammyControl::BuildDataType arm2_builder {"arm1_to_arm2_joint", joint_state_interface, position_joint_interface};
  DammyControl arm2_control {arm2_builder};
  DammyControl::BuildDataType effector_base_builder {"arm2_to_effector_base_joint", joint_state_interface, position_joint_interface};
  DammyControl effector_base_control {effector_base_builder};
  DammyControl::BuildDataType effector_end_builder {"effector_base_to_effector_end_joint", joint_state_interface, position_joint_interface};
  DammyControl effector_end_control {effector_end_builder};

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

inline ICSControl::ICSControl(BuildDataType& build_data, const std::string& device_path, const ics::ID& id)
  : data_ {build_data.joint_name_}
{
  registerJoint(data_, build_data);
}

inline void ICSControl::fetch()
{
}

inline void ICSControl::move()
{
}

inline DammyControl::DammyControl(BuildDataType& build_data)
  : data_ {build_data.joint_name_}
{
  registerJoint(data_, build_data);
}

inline void DammyControl::fetch()
{
  data_.pos_ = data_.cmd_;
}

inline void DammyControl::move()
{
}

inline Arcsys2HW::Arcsys2HW(hardware_interface::JointStateInterface* jnt_stat)
  : controls {}
{
  registerInterface(jnt_stat);
}

inline void Arcsys2HW::registerControl(JointControlInterface* jnt_cntr)
{
  static auto inserter = controls.begin();
  if (inserter == controls.cend()) throw std::out_of_range {"Too many JointControl"};
  *inserter = jnt_cntr;
  ++inserter;
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

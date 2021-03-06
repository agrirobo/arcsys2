#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <nav_msgs/Odometry.h>

#include <ics3/ics>

struct JointData
{
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
};

template<class JntCmdIF>
struct JointControlBuildData
{
  std::string joint_name_;
  hardware_interface::JointStateInterface& jnt_stat_;
  JntCmdIF& jnt_cmd_;
};

class JointControlInterface
{
public:
  virtual void fetch() = 0;
  virtual void move() = 0;
  virtual ~JointControlInterface() noexcept {}
};

class ICSControl
  : public JointControlInterface
{
public:
  using JntCmdType = hardware_interface::PositionJointInterface;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  ICSControl(BuildDataType&, ics::ICS3&, const ics::ID&);
  void fetch() override;
  void move() override;
private:
  JointData data_;
  ics::ICS3& driver_;
  ics::ID id_;
};

class DCMotorControl
  : public JointControlInterface
{
public:
  using JntCmdType = hardware_interface::VelocityJointInterface;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  DCMotorControl(BuildDataType&);
  void fetch() override;
  void move() override;
  void odomCb(const nav_msgs::OdometryConstPtr&);
private:
  JointData data_;
  double last_pos_;
  double last_vel_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
};

template<class JntCmdIF>
class DummyControl
  : public JointControlInterface
{
public:
  using JntCmdType = JntCmdIF;
  using BuildDataType = JointControlBuildData<JntCmdType>;
  DummyControl(BuildDataType&);
  void fetch() override;
  void move() override;
private:
  JointData data_;
};

using DummyPositionControl = DummyControl<hardware_interface::PositionJointInterface>;
using DummyVelocityControl = DummyControl<hardware_interface::VelocityJointInterface>;

class Arcsys2HW
  : public hardware_interface::RobotHW
{
public:
  static constexpr std::size_t JOINT_COUNT {6};
  using JointControlContainer = std::array<JointControlInterface*, JOINT_COUNT>;
  Arcsys2HW(hardware_interface::JointStateInterface*);
  void registerControl(JointControlInterface*);
  void read();
  void write();
  ros::Time getTime();
  ros::Duration getPeriod();
private:
  JointControlContainer controls;
};

template<class JntCmdIF>
void registerJoint(
    JointData&,
    hardware_interface::JointStateInterface&,
    JntCmdIF&);

template<class JntCmdIF>
void registerJoint(
    JointData&,
    JointControlBuildData<JntCmdIF>&);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arcsys2_control_node");
  ros::NodeHandle pnh {"~"};

  hardware_interface::JointStateInterface joint_state_interface {};
  hardware_interface::PositionJointInterface position_joint_interface {};

  DummyPositionControl::BuildDataType shaft_builder {"rail_to_base_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl shaft_control {shaft_builder};
  DummyPositionControl::BuildDataType arm0_builder {"shaft_to_arm0_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl arm0_control {arm0_builder};
  DummyPositionControl::BuildDataType arm1_builder {"arm0_to_arm1_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl arm1_control {arm1_builder};
  DummyPositionControl::BuildDataType arm2_builder {"arm1_to_arm2_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl arm2_control {arm2_builder};
  DummyPositionControl::BuildDataType effector_base_builder {"arm2_to_effector_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl effector_base_control {effector_base_builder};
  DummyPositionControl::BuildDataType effector_end_builder {"effector_to_blade_joint", joint_state_interface, position_joint_interface};
  DummyPositionControl effector_end_control {effector_end_builder};

  Arcsys2HW robot {&joint_state_interface};
  robot.registerInterface(&position_joint_interface);
  robot.registerControl(&shaft_control);
  robot.registerControl(&arm0_control);
  robot.registerControl(&arm1_control);
  robot.registerControl(&arm2_control);
  robot.registerControl(&effector_base_control);
  robot.registerControl(&effector_end_control);
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

inline ICSControl::ICSControl(BuildDataType& build_data, ics::ICS3& driver, const ics::ID& id)
  : data_ {build_data.joint_name_},
    driver_(driver), // for ubuntu 14.04
    id_ {id}
{
  registerJoint(data_, build_data);
}

inline void ICSControl::fetch()
{
  data_.pos_ = driver_.move(id_, ics::Angle::newRadian(data_.cmd_));
}

inline void ICSControl::move()
{
  driver_.move(id_, ics::Angle::newRadian(data_.cmd_));
}

inline DCMotorControl::DCMotorControl(BuildDataType& build_data)
  : data_ {build_data.joint_name_},
    last_pos_ {},
    last_vel_ {},
    nh_ {build_data.joint_name_},
    pub_ {nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1)},
    sub_ {nh_.subscribe("odom", 1, &DCMotorControl::odomCb, this)}
{
}

inline void DCMotorControl::fetch()
{
  data_.pos_ = last_pos_;
  data_.vel_ = last_vel_;
}

inline void DCMotorControl::move()
{
  geometry_msgs::Twist msg {};
  msg.linear.x = data_.cmd_;
  pub_.publish(std::move(msg));
}

inline void DCMotorControl::odomCb(const nav_msgs::OdometryConstPtr& odom)
{
  last_pos_ = odom->pose.pose.position.x;
  last_vel_ = odom->twist.twist.linear.x;
}

template<class JntCmdIF>
inline DummyControl<JntCmdIF>::DummyControl(BuildDataType& build_data)
  : data_ {build_data.joint_name_}
{
  registerJoint(data_, build_data);
}

template<>
inline void DummyPositionControl::fetch()
{
  data_.pos_ = data_.cmd_;
}

template<>
inline void DummyVelocityControl::fetch()
{
  data_.pos_ += data_.cmd_ * 0.01; // FIXME: test code
  data_.vel_ = data_.cmd_;
}

template<class JntCmdIF>
inline void DummyControl<JntCmdIF>::move()
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

inline void Arcsys2HW::read()
{
  for (auto control : controls) control->fetch();
}

inline void Arcsys2HW::write()
{
  for (auto control : controls) control->move();
}

inline ros::Time Arcsys2HW::getTime()
{
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod()
{
  return ros::Duration {0.01};
}

template<class JntCmdIF>
inline void registerJoint(
    JointData& joint,
    hardware_interface::JointStateInterface& jnt_stat,
    JntCmdIF& jnt_cmd)
{
  jnt_stat.registerHandle(hardware_interface::JointStateHandle {joint.name_, &joint.pos_, &joint.vel_, &joint.eff_});
  jnt_cmd.registerHandle(hardware_interface::JointHandle {jnt_stat.getHandle(joint.name_), &joint.cmd_});
}

template<class JntCmdIF>
inline void registerJoint(
    JointData& joint,
    JointControlBuildData<JntCmdIF>& build_data)
{
  registerJoint(joint, build_data.jnt_stat_, build_data.jnt_cmd_);
}

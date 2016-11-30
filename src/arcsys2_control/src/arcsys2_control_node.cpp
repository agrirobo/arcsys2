#include <array>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class JointControlInterface
{
public:
  virtual void fetch() = 0;
  virtual void move() = 0;
};

struct JointDatas {
  std::string name_;
  double cmd_;
  double pos_;
  double vel_;
  double eff_;
};

template<class CmdJntInterface>
void registerJoint(JointDatas&, hardware_interface::JointStateInterface&, CmdJntInterface&);

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

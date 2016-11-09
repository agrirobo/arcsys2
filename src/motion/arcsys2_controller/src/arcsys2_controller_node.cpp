#include"ros/ros.h"
#include"arm_msgs/ArmAnglesDegree.h"
#include"controller_manager/controller_manager.h"
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
  // for RobotHW
  hardware_interface::JointStateInterface jntStateInterface;
  hardware_interface::PositionJointInterface jntPosInterface;
  double servo_cmd[2];
  double servo_pos[2];
  double servo_vel[2];
  double servo_eff[2];
  // for real move
  ros::Publisher arm_motor_pub;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "arcsys2_controller_node");
  ros::NodeHandle nh;

  Arcsys2HW robot;
  controller_manager::ControllerManager cm(&robot);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
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
: jntStateInterface(),
  jntPosInterface(),
  arm_motor_pub()
{
  // [input] connect and register the joint state interface
  hardware_interface::JointStateHandle stateHandle0to1("arm0->arm1", &servo_pos[0], &servo_vel[0], &servo_eff[0]);
  jntStateInterface.registerHandle(stateHandle0to1);

  hardware_interface::JointStateHandle stateHandle1to2("arm1->arm2", &servo_pos[1], &servo_vel[1], &servo_eff[1]);
  jntStateInterface.registerHandle(stateHandle1to2);

  registerInterface(&jntStateInterface);

  // [output] connect and register the joint position interface
  hardware_interface::JointHandle posHandle0to1(jntStateInterface.getHandle("arm0->arm1"), &servo_cmd[0]);
  jntPosInterface.registerHandle(posHandle0to1);

  hardware_interface::JointHandle posHandle1to2(jntStateInterface.getHandle("arm1->arm2"), &servo_cmd[1]);
  jntPosInterface.registerHandle(posHandle1to2);

  registerInterface(&jntPosInterface);

  // for real move
  ros::NodeHandle nh;
  arm_motor_pub = nh.advertise<arm_msgs::ArmAnglesDegree>("arm_roll", 3);
}

inline void Arcsys2HW::read() {
  // TODO: read encoder
}

inline void Arcsys2HW::write() {
  arm_msgs::ArmAnglesDegree arm_msg;
  arm_msg.angles.push_back(servo_cmd[0]);
  arm_msg.angles.push_back(servo_cmd[1]);
  arm_motor_pub.publish(arm_msg);
  // TODO: write base motor
  // TODO: receive KRS angles
}

inline ros::Time Arcsys2HW::getTime() const {
  return ros::Time::now();
}

inline ros::Duration Arcsys2HW::getPeriod() const {
  return ros::Duration(0.01);
}

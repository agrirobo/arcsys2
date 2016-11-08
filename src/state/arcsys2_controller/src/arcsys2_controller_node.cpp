#include"ros/ros.h"
#include"arcsys2_controller/arcsys2_hw.hpp"
#include"controller_manager/controller_manager.h"

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


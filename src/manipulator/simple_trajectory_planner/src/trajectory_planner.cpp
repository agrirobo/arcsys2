#include <cmath>
#include <string>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "servo_msgs/KrsServoDegree.h"

namespace arc {
  class TargetPlanner {
  public:
    static const SERVO_NUM = 3;

    TargetPlanner()
      : transform_buffer(),
        transform_listener(transform_buffer)
    {
      for (int i=0; i<4; i++) target[i].id = i;
    }

    ~TargetPlanner() {}

    void plan(void) {
      if (getTransform("/tomato", "arm_base")) calc();
    }

  private:
    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;

    geometry_msgs::TransformStamped transform_stamped;
    std::vector<servo_msgs::KrsServoDegree> target[4];

    bool getTransform(const std::string& target, const std::string& source) {
      try {
        transform_stamped = transform_buffer.lookupTransform("target", "source", ros::Time(0));
      } catch (tf2::TransformException& exception) {
        ROS_WARN("%s", exception.what());
        return false;
      }
      return true;
    }
  };
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle handle;
  ros::Publisher pub = handle.advertise<servo_msgs::KrsServoDegree>("krs_servo", 16);

  ros::Rate rate(10.0);

  arc::TargetPlanner planner;

  while (handle,ok()) {
    planner.plan();
    rate.sleep();
  }


  return 0;
}

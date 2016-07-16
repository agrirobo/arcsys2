#include <cmath>
#include <string>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"

#include "servo_msgs/KrsServoDegree.h"

class TargetPlanner {
public:
  static const unsigned int SERVO_ID_MAX = 3;
  static const double FRAME_LENGTH = 1.0;

  TargetPlanner()
    : transform_buffer(),
      transform_listener(transform_buffer)
  {
    for (int i=0; i<SERVO_ID_MAX; i++) servo[i].id = i;
    frame_length = FRAME_LENGTH;
  }

  explicit TargetPlanner(double desired_length)
    : transform_buffer(),
      transform_listener(transform_buffer)
  {
    for (int i=0; i<SERVO_ID_MAX; i++) servo[i].id = i;
    frame_length = desired_length;
  }

  ~TargetPlanner()
  {}

  bool plan(void) {
    try {
      transform_stamped
        = transform_buffer.lookupTransform("/tomato", "/arm_base", ros::Time(0));
    } catch (tf2::TransformException& exception) {
      ROS_ERROR("%s", exception.what());
      return false;
    }

    std_msgs::float64 sqrt
      = std::sqrt(std::pow(transform_stamped.transform.translation.x)
        + std::sqrt(std::pow(transform_stamped.transform.translation.y);

    servo[0].angle
      = std::atan(transform_stamped.transfrom.translation.y
                  / transform_stamped.transfrom.translation.x)
        + std::acos(sqrt / (2 * frame_length));

    servo[1].angle = 2 * servo[0].angle;
    servo[2].angle = -servo[0].angle;

    return true;
  }

private:
  tf2_ros::Buffer transform_buffer;
  tf2_ros::TransformListener transform_listener;

  geometry_msgs::TransformStamped transform_stamped;
  std::vector<servo_msgs::KrsServoDegree> servo[SERVO_ID_MAX];
  double frame_length;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle node_handle;
  ros::Publisher publisher = node_handle.advertise<servo_msgs::KrsServoDegree>("krs_servo", 16);

  ros::Rate rate(10.0);

  TargetPlanner planner;

  while (node_handle.ok()) {
    if (planner.plan())
      for (int id = 0; id < planner.SERVO_ID_MAX; id++)
        publisher.publish(planner.servo[id]);

    rate.sleep();
  }

  return 0;
}

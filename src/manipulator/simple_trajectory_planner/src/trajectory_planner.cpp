#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Quaternion>
#include <geometry_msgs/TransformStamped.h>

namespace arc {
class GoalPlanner {
public:
  explicit GoalPlanner()
  {

  }

  virtual ~GoalPlanner() {}


private:

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_planner");
  ros::NodeHandle handle;

  

  return 0;
}

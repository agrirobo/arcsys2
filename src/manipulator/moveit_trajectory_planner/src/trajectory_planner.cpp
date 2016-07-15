#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

namespace arc {
  class ScaraRobot {
  public:
    ScaraRobot() {
      length = {100, 200, 200, 200};
      theta = {0.0, 0.0, 0.0, 0.0};
    }

    ScaraRobot(const std::vector<int>& arg_length, const std::vector<float>& arg_theta) {
      for (int i = 0; i < arg_length.size(); i++) length.push_back(arg_length[i]);
      for (int i = 0; i < arg_theta.size(); i++) theta.push_back(arg_theta[i]);
    }

    ~ScaraRobot() {
      std::cout << "dtor" << std::endl;
    }

  private:
    std::vector<int> length; // [mm]
    std::vector<float> theta; // [rad]
  };
}

class TrajectoryPlanner {
public:
  TrajectoryPlanner()
    : state_src(), state_dest(), handle()
  {
    sub = handle.subscribe("tomato_point", 1, plannerCallback, this);
    pub = handle.advertise<geometry_msgs::PoseArray>("planner", 1);
  }

  TrajectoryPlanner(ros::NodeHandle& arg_handle)
    : state_src(), state_dest()
  {
    sub = arg_handle.subscribe("tomato_point", 1, plannerCallback, this);
    pub = arg_handle.advertise<geometry_msgs::PoseArray>("planner", 1);
  }

  ~TrajectoryPlanner() {
    std::cout << "dtor" << std::endl;
  }

  arc::ScaraRobot convertPointToJointStatus(const geometry_msgs::Point::ConstPtr& point) {

  }

private:
  void plannerCallback(const geometry_msgs::Point::ConstPtr& point) {
    std::cout << "callback" << std::endl;
  }

  ros::NodeHandle handle;
  ros::Subscriber sub;
  ros::Publisher pub;
  arc::ScaraRobot state_src, state_dest;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_planner_node");

  ros::NodeHandle handle;
  TrajectoryPlanner planner(handle);

  return 0;
}

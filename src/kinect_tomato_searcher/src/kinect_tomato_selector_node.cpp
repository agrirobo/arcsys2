#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <kinect_tomato_searcher/CalibrationConfig.h>

#include <algorithm>
#include <memory>
#include <vector>

struct Calibrator
{
  Calibrator(double offset_x, double offset_y, double offset_z);
  double offset_x_;
  double offset_y_;
  double offset_z_;

  geometry_msgs::Pose calibrate(geometry_msgs::Pose) const;
private:
  dynamic_reconfigure::Server<kinect_tomato_searcher::CalibrationConfig> server;
  dynamic_reconfigure::Server<kinect_tomato_searcher::CalibrationConfig>::CallbackType f;
  void callback(kinect_tomato_searcher::CalibrationConfig& config, uint32_t level);
};

class NearSelector
{
public:
  geometry_msgs::Pose select(const geometry_msgs::PoseArray& pose_array);
private:
  geometry_msgs::Pose last_pose_;
};

void callback(const geometry_msgs::PoseArray&);
double calcDiff(const geometry_msgs::Pose&, const geometry_msgs::Pose&);

ros::Publisher pub;
std::unique_ptr<Calibrator> calibrator;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_tomato_selector_node");
  ros::NodeHandle tomato_nh {"tomato"};
  auto sub = tomato_nh.subscribe("array", 1, &callback);
  pub = tomato_nh.advertise<geometry_msgs::PoseStamped>("raw", 1);
  ros::NodeHandle offset_nh {"~offset"};
  double offset_x, offset_y, offset_z;
  offset_nh.getParam("x", offset_x);
  offset_nh.getParam("y", offset_y);
  offset_nh.getParam("z", offset_z);
  calibrator = std::unique_ptr<Calibrator> {new Calibrator {offset_x, offset_y, offset_z}};
  ros::spin();
  return 0;
}

Calibrator::Calibrator(double offset_x, double offset_y, double offset_z)
  : offset_x_ {offset_x},
    offset_y_ {offset_y},
    offset_z_ {offset_z},
    server {},
    f(std::bind(&Calibrator::callback, this, std::placeholders::_1, std::placeholders::_2))
{
   server.setCallback(f);
}

inline geometry_msgs::Pose Calibrator::calibrate(geometry_msgs::Pose pose) const
{
  pose.position.x += offset_x_;
  pose.position.y += offset_y_;
  pose.position.z += offset_z_;
  return pose;
}

void Calibrator::callback(kinect_tomato_searcher::CalibrationConfig& config, uint32_t level)
{
  offset_x_ = config.offset_x;
  offset_y_ = config.offset_y;
  offset_z_ = config.offset_z;
}

inline geometry_msgs::Pose NearSelector::select(const geometry_msgs::PoseArray& pose_array)
{
  std::vector<double> diffs(pose_array.poses.size());
  auto diff_it = diffs.begin();
  for (auto pose : pose_array.poses) {
    *diff_it = calcDiff(pose, last_pose_);
    ++diff_it;
  }
  auto min_diff_it = std::min_element(diffs.cbegin(), diffs.cend());
  return pose_array.poses[min_diff_it - diffs.cbegin()];
}

/**
 * This function calculate lie distance.
 * 
 * @return diff linear difference. NOT real difference.
 */
inline double calcDiff(const geometry_msgs::Pose& target, const geometry_msgs::Pose& origin) {
  double diff_x {target.position.x - origin.position.x};
  double diff_y {target.position.y - origin.position.y};
  double diff_z {target.position.z - origin.position.z};
  return diff_x + diff_y + diff_z;
}

void callback(const geometry_msgs::PoseArray& pose_array)
{
  static NearSelector selector {};
  geometry_msgs::PoseStamped pub_msg;
  pub_msg.header = pose_array.header;
  pub_msg.pose = calibrator->calibrate(selector.select(pose_array));
  pub.publish(pub_msg);
}

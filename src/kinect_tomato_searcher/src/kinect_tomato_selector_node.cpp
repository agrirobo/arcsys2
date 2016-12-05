#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <vector>

ros::Publisher pub;

class NearSelector
{
public:
  geometry_msgs::Pose select(const geometry_msgs::PoseArray& pose_array);
private:
  geometry_msgs::Pose last_pose_;
};

void callback(const geometry_msgs::PoseArray&);
double calcDiff(const geometry_msgs::Pose&, const geometry_msgs::Pose&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_tomato_selector_node");
  ros::NodeHandle tomato_nh {"tomato"};
  auto sub = tomato_nh.subscribe("array", 1, &callback);
  pub = tomato_nh.advertise<geometry_msgs::PoseStamped>("raw", 1);
  ros::spin();
  return 0;
}

inline geometry_msgs::Pose NearSelector::select(const geometry_msgs::PoseArray& pose_array)
{
  std::vector<double> diffs(pose_array.poses.size());
  auto diff_it {diffs.begin()};
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
  pub_msg.pose = selector.select(pose_array);
  pub.publish(pub_msg);
}

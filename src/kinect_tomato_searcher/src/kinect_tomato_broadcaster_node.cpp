#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

class TomatoBroadcaster {
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped transform_;
  ros::Subscriber sub_;

public:
  TomatoBroadcaster(ros::NodeHandle& node_handle)
    : broadcaster_ {},
      transform_ {},
      sub_ {node_handle.subscribe<geometry_msgs::PoseStamped>("tomato_point", 1, &TomatoBroadcaster::callback, this)}
  {
  }

private:
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    transform_.header = msg->header;
    transform_.header.frame_id = "kinect";
    transform_.child_frame_id = "tomato";

    transform_.transform.translation.x = msg->pose.position.x;
    transform_.transform.translation.y = msg->pose.position.y;
    transform_.transform.translation.z = msg->pose.position.z;
    transform_.transform.rotation = msg->pose.orientation;

    broadcaster_.sendTransform(transform_);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "kinect_tomato_broadcaster_node");
  ros::NodeHandle node_handle;

  TomatoBroadcaster broadcaster {node_handle};

  ros::spin();

  return 0;
}

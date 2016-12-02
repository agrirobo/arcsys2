#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

class TomatoBroadcaster {
  tf2_ros::TransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped transform_;
  ros::Subscriber sub_;

public:
  TomatoBroadcaster(ros::NodeHandle node_handle)
    : transform_ {},
      sub_ {node_handle.subscribe<geometry_msgs::PointStamped>("tomato_point/trusted", 1, &TomatoBroadcaster::callback, this)}
  {
  }

private:
  void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    transform_.header = msg->header;
    transform_.header.frame_id = "kinect";
    transform_.child_frame_id = "tomato";

    transform_.transform.translation.x = msg->point.x;
    transform_.transform.translation.y = msg->point.y;
    transform_.transform.translation.z = msg->point.z;
    transform_.transform.rotation.w = 1.0;

    broadcaster_.sendTransform(transform_);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "tomato_point_broadcaster");
  ros::NodeHandle node_handle;

  TomatoBroadcaster broadcaster {node_handle};

  ros::spin();

  return 0;
}

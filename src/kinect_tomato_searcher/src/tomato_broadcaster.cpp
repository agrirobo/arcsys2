#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

int offset_x, offset_y, offset_z;

void pointCallback(const geometry_msgs::Point::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transform_stamped;
  
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "kinect";
  transform_stamped.child_frame_id = "tomato";
  transform_stamped.transform.translation.x = msg->x + offset_x;
  transform_stamped.transform.translation.y = msg->y + offset_y;
  transform_stamped.transform.translation.z = msg->z + offset_z;
  transform_stamped.transform.rotation.x = 0;
  transform_stamped.transform.rotation.y = 0;
  transform_stamped.transform.rotation.z = 0;
  transform_stamped.transform.rotation.w = 1;

  br.sendTransform(transform_stamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tomato_broadcaster");
  ros::NodeHandle node;
  ros::NodeHandle offset_nh {"~offset"};

  offset_nh.getParam("x", offset_x);
  offset_nh.getParam("y", offset_y);
  offset_nh.getParam("z", offset_z);

  ros::Subscriber sub = node.subscribe("target_point", 1, &pointCallback);

  ros::spin();
  return 0;
}

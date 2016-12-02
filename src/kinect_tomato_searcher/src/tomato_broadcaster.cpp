#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

int offsetX, offsetY, offsetZ;

void pointCallback(const geometry_msgs::Point::ConstPtr& msg) {
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "kinect";
  transformStamped.child_frame_id = "tomato";
  transformStamped.transform.translation.x = msg->x + offsetX;
  transformStamped.transform.translation.y = msg->y + offsetY;
  transformStamped.transform.translation.z = msg->z + offsetZ;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tomato_broadcaster");
  ros::NodeHandle node;
  ros::NodeHandle private_node {"~"};

  private_node.getParam("offset_x", offsetX);
  private_node.getParam("offset_y", offsetY);
  private_node.getParam("offset_z", offsetZ);

  ros::Subscriber sub = node.subscribe("target_point", 1, &pointCallback);

  ros::spin();
  return 0;
}

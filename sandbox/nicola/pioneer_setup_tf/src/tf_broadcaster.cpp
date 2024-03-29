#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "pioneer_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(50);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1905, 0.0, 0.2659)),
        ros::Time::now(),"/base_link", "/base_laser"));
    ros::spinOnce();
    r.sleep();
  }
}


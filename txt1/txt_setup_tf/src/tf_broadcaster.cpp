/*
 * tf_broadcaster.cpp
 *
 *  Created on: Jan 15, 2011
 *      Author: root
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "txt_setup_tf");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
	    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.14605)),
	        ros::Time::now(),"base_footprint", "base_link"));

	    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1905, 0.0, 0.1435)),
	        ros::Time::now(),"base_link", "base_laser_link"));

	    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1448, 0.0, 0.2832)),
	        ros::Time::now(),"base_link", "base_camera_link"));

	    broadcaster.sendTransform(
	      tf::StampedTransform(
	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.1016, 0.0, 0.3187)),
	        ros::Time::now(),"base_link", "base_imu_link"));

	    broadcaster.sendTransform(
	    	      tf::StampedTransform(
	    	        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.0762, 0.0, 0.3492)),
	    	        ros::Time::now(),"base_link", "base_gps_link"));

    r.sleep();
  }
  return 0;
}

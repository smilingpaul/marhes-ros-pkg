#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "stage_light_ml/move_robot.h"
#include "tf/tf.h"
#include "std_msgs/Bool.h"
#include <math.h>

class MovePotField
{
public:
  MovePotField(ros::NodeHandle n)
  {
    n_ = n;
    odom_sub_ = n.subscribe("/odom", 10, &MovePotField::odom_cb, this);
    move_srv_ = n_.advertiseService("/move_robot", &MovePotField::move_cb, this);
    laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("/scan_laser", 10, &MovePotField::laser_cb, this);
    bool_pub_ = n_.advertise<std_msgs::Bool>("/move_done", 1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //cmd_moving_ = true;
    vx_max_ = 1.5;
    vt_max_ = 1.0;
    kgoal_ = 0.1;
    kobs_ = 0.5;
    kv_ = 1;
    kt_ = 0.5;
    resolution_ = 0.5;
    dist_limit_ = 1.0;
  }
private:
  ros::NodeHandle n_;
  ros::Publisher bool_pub_;
  ros::Subscriber odom_sub_;
  ros::Publisher vel_pub_;
  ros::ServiceServer move_srv_;
  ros::Subscriber laser_sub_;
  sensor_msgs::LaserScan ls_;
  nav_msgs::Odometry odom_;
  bool cmd_moving_;
  float kgoal_, kobs_, kv_, kt_, vx_max_, vt_max_, dist_limit_;
  double goal_x_, goal_y_, resolution_;

  bool move_cb(stage_light_ml::move_robot::Request &req,
               stage_light_ml::move_robot::Response &res )
  {
    cmd_moving_ = true;
    goal_x_ = req.pose.position.x;
    goal_y_ = req.pose.position.y;
    res.result = true;
    return true;
  }

  void laser_cb(sensor_msgs::LaserScan msg)
  {
    ls_ = msg;
	float ang_min, ang_max, ang_inc, dist_min, dist_max, obs_x, obs_y, ang_diff,
		dx, dy, test_x, test_y, dist_smallest, last_pt, dist_current;
	float v[8] = {0};
	int num_points;
	std::vector<float> ranges;
	geometry_msgs::Twist vel_msg;

	if (cmd_moving_)
	{
	  ang_min = ls_.angle_min;
	  ang_max = ls_.angle_max;
	  ang_inc = ls_.angle_increment;
	  dist_min = ls_.range_min;
	  dist_max = ls_.range_max;
	  num_points = (ang_max - ang_min) / ang_inc + 1;
	  ranges = ls_.ranges;

	  for (int i = 0; i < 8; i++)
	  {
		switch(i)
		{
			case 0:
				test_x = odom_.pose.pose.position.x - resolution_;
				test_y = odom_.pose.pose.position.y + resolution_;
				break;
			case 1:
				test_x = odom_.pose.pose.position.x;
				test_y = odom_.pose.pose.position.y + resolution_;
				break;
			case 2:
				test_x = odom_.pose.pose.position.x + resolution_;
				test_y = odom_.pose.pose.position.y + resolution_;
				break;
			case 3:
				test_x = odom_.pose.pose.position.x - resolution_;
				test_y = odom_.pose.pose.position.y;
				break;
			case 4:
				test_x = odom_.pose.pose.position.x + resolution_;
				test_y = odom_.pose.pose.position.y;
				break;
			case 5:
				test_x = odom_.pose.pose.position.x - resolution_;
				test_y = odom_.pose.pose.position.y - resolution_;
				break;
			case 6:
				test_x = odom_.pose.pose.position.x;
				test_y = odom_.pose.pose.position.y - resolution_;
				break;
			case 7:
				test_x = odom_.pose.pose.position.x + resolution_;
				test_y = odom_.pose.pose.position.y - resolution_;
				break;
		}

		v[i] = 0.5 * kgoal_ * pow(dist(test_x, test_y, goal_x_, goal_y_), 2);

//		for (int pt = 0; pt < (int)ranges.size(); pt++)
//		{
//		  if (ranges[pt] < 5)
//		  {
//			ang_diff = tf::getYaw(odom_.pose.pose.orientation) - (ang_min + ang_inc * pt);
//			obs_x = ranges[pt] * cos(ang_diff) + odom_.pose.pose.position.x;
//			obs_y = ranges[pt] * sin(ang_diff) + odom_.pose.pose.position.y;
//			dist_current = dist(test_x, test_y, obs_x, obs_y);
//
//			if (pt == 0)
//			{
//				dist_smallest = dist_current;
//			}
//			else if (last_pt == pt - 1)
//			{
//				if (dist_current < dist_smallest)
//					dist_smallest = dist_current;
//			}
//			else
//			{
//				v[i] += kobs_ / dist_smallest;
//				dist_smallest = dist_current;
//			}
//
//			last_pt = pt;
//		  }
//		}
	  }

	  //ROS_INFO("0: %f, 1: %f, 2: %f, 3: %f, 4: %f, 5: %f, 6: %f, 7: %f", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);

	  // Calculate the sobel gradient
	  dx = -0.25 * (-(v[0] + 2 * v[3] + v[5]) + v[2] + 2 * v[4] + v[7]);
	  dy = -0.25 * (-(v[5] + 2 * v[6] + v[7]) + v[0] + 2 * v[1] + v[2]);

	  vel_msg.linear.x = kv_ * sqrt(dx * dx + dy * dy);
	  vel_msg.angular.z =  -kt_ * vel_msg.linear.x * (tf::getYaw(odom_.pose.pose.orientation) - atan2(dy, dx));

	  if (vel_msg.linear.x > vx_max_)
		  vel_msg.linear.x = vx_max_;
	  if (vel_msg.angular.z > vt_max_)
		  vel_msg.angular.z = vt_max_;

	  vel_pub_.publish(vel_msg);

	  if (dist(odom_.pose.pose.position.x, odom_.pose.pose.position.y, goal_x_, goal_y_) < dist_limit_)
	  {
		  std_msgs::Bool bool_msg;
		  bool_msg.data = true;
		  cmd_moving_ = false;
		  bool_pub_.publish(bool_msg);
	  }
	}
  }

  float dist(float x, float y, float x_goal, float y_goal)
  {
	float distance = sqrt(pow(x - x_goal, 2) + pow(y - y_goal, 2));
	return distance;
  }

  void odom_cb(nav_msgs::Odometry msg)
  {
    odom_ = msg;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "stage_move");
  ros::NodeHandle n;
  MovePotField* mpf = new MovePotField(n);
  ros::spin();
  return 0;
}

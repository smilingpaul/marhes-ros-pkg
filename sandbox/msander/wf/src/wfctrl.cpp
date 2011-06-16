#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "wf/Dist.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "p2os_driver/MotorState.h"



class WF_Agent{
public:
        WF_Agent(ros::NodeHandle nh);
        //virtual ~WF_Agent();
        
        ros::Publisher vel_pub;
	ros::Publisher mot_pub;

  	std::string motor_state_topic;
        
        float min_dist;
        float angle_to_min;
	float cmd_dist;
        
private:
        void odom_cb(nav_msgs::Odometry odom_msg);
        void dist_cb(wf::Dist dist_msg);
        void laser_cb(sensor_msgs::LaserScan laser_msg);

/*ros::NodeHandle provides automatic startup and shutdown of the internal
 *node inside a roscpp program.
 */
        ros::NodeHandle n;       
        ros::Subscriber odom_sub;
        ros::Subscriber laser_sub;
        ros::Publisher vis_pub;
	ros::Subscriber wall_dist_sub;


        
  	sensor_msgs::LaserScan cur_laser_dat;
	wf::Dist cur_cmd_dist;
        float angle_min;        // start angle of the scan [rad]
        float angle_max;        // end angle of the scan [rad]
        float angle_incr;       // angular distance between measurements [rad]

        float time_incr;        // time between measurements [seconds] - if your scanner
                                 // is moving, this will be used in interpolating position
                                 // of 3d points
        float scan_time;        // time between scans [seconds]

        float range_min;        // minimum range value [m]
        float range_max;        // maximum range value [m]

        std::vector<float> ranges;         // range data [m] (Note: values < range_min or > range_max should be discarded)
        
        float i_min;
	float x_wallmin_laser;
	float y_wallmin_laser;
        
  	std::string odom_topic, vel_topic, laser_topic, dist_topic;
	visualization_msgs::Marker marker;


//        int linAxis, angAxis;
};


//Initialize parameters
WF_Agent::WF_Agent(ros::NodeHandle nh):
n(nh) //ros::NodeHandle n = nh;
{
        //accessing private parameters
        ros::NodeHandle n_private("~");
       
        /*linAxis, angAxis variables are used to define which axes of the
         *joystick will control the pioneer robot. Also, the parameter server
         * was checked for new scalar values for driving the robot. */
//        n_private.param("linAxis", linAxis, 1);
//        n_private.param("angAxis", angAxis, 0);
 	n_private.param("vel_topic", vel_topic, std::string("cmd_vel"));
//	n_private.param("odom_topic", odom_topic, std::string("odom"));
// 	n_private.param("laser_topic", laser_topic, std::string("base_scan"));
 	n_private.param("odom_topic", odom_topic, std::string("pose"));
 	n_private.param("laser_topic", laser_topic, std::string("scan"));
 	n_private.param("dist_topic", dist_topic, std::string("dist"));
	n_private.param("motor_state_topic", motor_state_topic, std::string("cmd_motor_state")); //pioneer stuff
        
	/*creating a publisher that will advertise the cmd_vel(command velocity
         topic of the pioneer robot. */
    	vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
	vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  	mot_pub = n.advertise<p2os_driver::MotorState>(motor_state_topic.c_str(),1); //pioneer stuff
       
        /*subscribing to the joystick topic for the input to drive the robot
         *If the node is slow in processing incoming messages on the joystick topic,
         * up to 10 messages will be buffered before any are lost.   */
        odom_sub = n.subscribe<nav_msgs::Odometry>(odom_topic.c_str(), 10, &WF_Agent::odom_cb, this);
        laser_sub = n.subscribe<sensor_msgs::LaserScan>(laser_topic.c_str(), 10, &WF_Agent::laser_cb, this);
	wall_dist_sub = n.subscribe<wf::Dist>(dist_topic.c_str(), 10, &WF_Agent::dist_cb, this);


}


void WF_Agent::dist_cb(wf::Dist dist_msg)
{
	cmd_dist = dist_msg.dist;
}

// Generate debug message
void WF_Agent::odom_cb(nav_msgs::Odometry odom_msg)
{
  	ROS_INFO("PosX: %f, PosY: %f, LinVel: %f, AngVel: %f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
    	    odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z);
	
}

void WF_Agent::laser_cb(sensor_msgs::LaserScan laser_msg)
{
    cur_laser_dat = laser_msg;
    angle_min = laser_msg.angle_min;
    angle_incr = laser_msg.angle_increment;
    angle_max = laser_msg.angle_max;
    time_incr = laser_msg.time_increment;
    scan_time = laser_msg.scan_time;
    range_min = laser_msg.range_min;
    range_max = laser_msg.range_max;
    ranges = laser_msg.ranges;
  	
  	min_dist = range_max;
  	for (float i=0;i<ranges.size();i++)
  	{
  	    if (ranges[i] < min_dist && ranges[i] > range_min)
        {
  	        min_dist = ranges[i];
  	        i_min = i;
  	    }
  	}
  	angle_to_min = angle_min + angle_incr*i_min;

  	ROS_INFO("Bearing_to_min: %f, Range_min: %f, minangle: %f, minr: %f, maxr: %f", angle_to_min, min_dist, angle_min, range_min, range_max); 


// Display a marker at the minimum distance
	x_wallmin_laser = min_dist*cos(angle_to_min);
	y_wallmin_laser = min_dist*sin(angle_to_min);

	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Minimum_distance";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x_wallmin_laser;
	marker.pose.position.y = y_wallmin_laser;
	marker.pose.position.z = 0.1;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	vis_pub.publish( marker );

}

// ------- Find min dist and bearing
// find min dist in range[]
// determine index of min dist
// calculate angle to min dist as angle_min + index*angle_increment


int main(int argc, char **argv) 
{
	ros::init(argc, argv, "WF_Agent");
	ros::NodeHandle n;

        //a constructor function is called by dynamically allocating the object
        WF_Agent *p = new WF_Agent(n);

  	ros::Rate loop_rate(10);
	float v;
	float k1;
	float k2;
	float d_act;
	float d;
	float bearing;
	float omega_n;
	float damping_ratio;
	float d_ref;	

	v = 0.3;
	omega_n = 2;
	damping_ratio = 0.8;
	k1 = omega_n*omega_n;
	k2 = 2*damping_ratio*sqrt(omega_n);
	d_ref = .6;
	

	while(ros::ok())
	{
	    p2os_driver::MotorState mot_state_msg;
	    mot_state_msg.state = true;
	    p->mot_pub.publish(mot_state_msg);
	    
	    //d_ref = p->cmd_dist;
	
	    d_act = p->min_dist;
	    d = d_ref - d_act;
	    bearing = p->angle_to_min;
  	    //ROS_INFO("Bearing: %f", bearing);
	    //ROS_INFO("Commanded Distance: %f", d_ref);

	    geometry_msgs::Twist vel_msg;
	    if (d_act < 0.3)
	    {
	    	vel_msg.linear.x = 0;
	    	vel_msg.angular.z = 0;
	    }
	    else
	    {
	     vel_msg.linear.x = v;
	     vel_msg.angular.z = -k1*v*d*sin(-bearing+M_PI/2)/(-bearing+M_PI/2) - k2*v*(-bearing+M_PI/2);
	    }

	    p->vel_pub.publish(vel_msg);
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	return 0;
}



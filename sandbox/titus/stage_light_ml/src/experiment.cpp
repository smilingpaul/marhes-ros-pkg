/*
 * experiment.cpp
 *
 *  Created on: Apr 2, 2011
 *      Author: titus
 */

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "stage_light_ml/actions.h"
#include "stage_light_ml/qlearner.h"
#include "stage_light_ml/states_light.h"
#include "stage_light_ml/move_robot.h"
#include "std_msgs/Bool.h"

nav_msgs::Odometry odom;
bool move_stopped = false;
//void odom_cb(nav_msgs::Odometry msg);
void bool_cb(const std_msgs::Bool msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "experiment");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	int num_reps, max_explore, cnt_rep = 0, state, state_p, action;
	double freq, start_x, start_y, radius, learning_rate, discount_factor, reward;
	bool learn;

	n_private.param("num_reps", num_reps, 1000);
	n_private.param("freq", freq, 1.0);
	n_private.param("start_x", start_x, -5.0);
	n_private.param("start_y", start_y, 5.0);
	n_private.param("goal_radius", radius, 0.5);
	n_private.param("learn", learn, true);
	n_private.param("learning_rate", learning_rate, 0.5);
	n_private.param("discount_factor", discount_factor, 0.5);
	n_private.param("max_explore", max_explore, 8);

	States* states = new States(n);
	Actions* actions = new Actions(n);
	QLearner * qobj = new QLearner(actions->GetNumActions(),
			states->GetNumStates(), learning_rate, discount_factor, learn,
			max_explore);

	ros::Subscriber bool_sub = n.subscribe("/move_done", 1, bool_cb);
//	ros::Subscriber odom_sub = n.subscribe("/odom", 10, odom_cb);
	ros::ServiceClient client = n.serviceClient<stage_light_ml::move_robot>("/move_robot");

	ros::Rate loop_rate(freq);

	while(ros::ok() && (cnt_rep < num_reps))
	{
		state = (int)states->GetState();

		ROS_INFO("Starting rep: %d", cnt_rep);

		while(ros::ok() && (states->GetDistance() > radius))
		{
			action = qobj->GetAction(state);
			actions->Move((Actions::moveType)action);
			state_p = (int)states->GetState();

			ROS_INFO("Action: %d, produced state: %d", action, state_p);

			if (learn)
			{
				reward = states->GetReward();
				qobj->Update(reward, state, state_p, action);

				ROS_INFO("Action: %d, produced state: %d with reward: %f", action, state_p, reward);
			}

			state = state_p;

			ros::spinOnce();
			loop_rate.sleep();
		}

		// Move to starting spot
		stage_light_ml::move_robot srv;
		srv.request.pose.position.x = start_x;
		srv.request.pose.position.y = start_y;
		client.call(srv);

		while(1)
		{
//			double dist = sqrt(pow(start_x - odom.pose.pose.position.x,2) +
//					pow(start_y - odom.pose.pose.position.y,2));
//			if (dist < radius)
//			if (odom.twist.twist.linear.x < 0.01)
//				break;
			if (move_stopped)
			{
				move_stopped = false;
				break;
			}
		}

		cnt_rep++;
	}

	return 0;
}

//void odom_cb(nav_msgs::Odometry msg)
//{
//	odom = msg;
//}

void bool_cb(const std_msgs::Bool msg)
{
	move_stopped = msg.data;
}

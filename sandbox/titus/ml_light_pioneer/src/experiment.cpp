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

class Experiment
{
public:
	Experiment(ros::NodeHandle n);
private:
	ros::NodeHandle n_;
	nav_msgs::Odometry odom;

	bool move_stopped_, learn_;
	int num_reps_, max_explore_, cnt_rep_, state_, state_p_, action_, mode_;
	double freq_, start_x_, start_y_, radius_, learning_rate_, discount_factor_, reward_;

	static const int MODE_REP_START = 0;
	static const int MODE_REP = 1;
	static const int MODE_RETURN = 2;
	static const int MODE_DONE = 3;

	States* states_;
	Actions* actions_;
	QLearner * qobj_;
	stage_light_ml::move_robot srv_;

	ros::Subscriber bool_sub_;
	//	ros::Subscriber odom_sub_;
	ros::ServiceClient client_;
	ros::Timer timer_;

	//void odom_cb(nav_msgs::Odometry msg);
	void bool_cb(const std_msgs::Bool msg);
	void timer_cb(const ros::TimerEvent& event);

};

Experiment::Experiment(ros::NodeHandle n):n_(n)
{
	ros::NodeHandle n_private("~");
	n_private.param("num_reps", num_reps_, 1000);
	n_private.param("freq", freq_, 1.0);
	n_private.param("start_x", start_x_, -5.0);
	n_private.param("start_y", start_y_, 5.0);
	n_private.param("goal_radius", radius_, 0.5);
	n_private.param("learn", learn_, true);
	n_private.param("learning_rate", learning_rate_, 0.5);
	n_private.param("discount_factor", discount_factor_, 0.5);
	n_private.param("max_explore", max_explore_, 8);

	cnt_rep_ = 0;
	move_stopped_ = false;
	mode_ = MODE_REP_START;

	states_ = new States(n);
	actions_ = new Actions(n);
	qobj_ = new QLearner(actions_->GetNumActions(),
			states_->GetNumStates(), learning_rate_, discount_factor_, learn_,
			max_explore_);

	bool_sub_ = n.subscribe("/move_done", 1, &Experiment::bool_cb, this);
//	ros::Subscriber odom_sub_ = n.subscribe("/odom", 10, odom_cb);
	client_ = n.serviceClient<stage_light_ml::move_robot>("/move_robot");
	timer_ = n.createTimer(ros::Duration(1/freq_), &Experiment::timer_cb, this);
}

void Experiment::bool_cb(const std_msgs::Bool msg)
{
	move_stopped_ = msg.data;
}

void Experiment::timer_cb(const ros::TimerEvent& event)
{
	switch(mode_)
	{
	case MODE_REP_START:
		state_ = (int)states_->GetState();
		ROS_INFO("Starting rep: %d", cnt_rep_);
		mode_ = MODE_REP;
		break;
	case MODE_REP:
		action_ = qobj_->GetAction(state_);
		actions_->Move((Actions::moveType)action_);
		state_p_ = (int)states_->GetState();

		ROS_INFO("Action: %d, produced state: %d", action_, state_p_);

		if (learn_)
		{
			reward_ = states_->GetReward();
			qobj_->Update(reward_, state_, state_p_, action_);
			ROS_INFO("Action: %d, produced state: %d with reward: %f", action_, state_p_, reward_);
			ROS_INFO("Table: \n%s", qobj_->PrintTable().c_str());
		}

		state_ = state_p_;

		if (states_->GetDistance() < radius_)
		{
			mode_ = MODE_RETURN;
			ROS_INFO("Completed rep: %d, returning to start location", cnt_rep_);

			// Move to starting spot
			srv_.request.pose.position.x = start_x_;
			srv_.request.pose.position.y = start_y_;
			client_.call(srv_);
		}
		break;
	case MODE_RETURN:
		if (move_stopped_ == true)
		{
			move_stopped_ = false;
			mode_ = MODE_REP_START;
			cnt_rep_++;
		}

		if (cnt_rep_ > num_reps_)
			mode_ = MODE_DONE;
		break;
	case MODE_DONE:
		break;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "experiment");
	ros::NodeHandle n;

	Experiment* e = new Experiment(n);
	ros::spin();

	return 0;
}

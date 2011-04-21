#include "stage_light_ml/qlearner.h"

QLearner::QLearner(int num_actions, int num_states, double learning_rate,
		double discount_factor, bool learn, int max_explore) :
		num_actions_(num_actions),
		num_states_(num_states),
		cnt_explore_(0),
		max_explore_(max_explore),
		learning_rate_(learning_rate),
		discount_factor_(discount_factor),
		learn_(learn)
{
	size_array_ = num_actions_ * num_states_;
	for (int i = 0; i < size_array_; i++)
		q_array_.push_back(0.0);
	Init();
	srand ( time(NULL) );
}

QLearner::QLearner(int num_actions, int num_states, double learning_rate,
		double discount_factor, bool learn, int max_explore,
		std::vector<double> q_array_init) :
		num_actions_(num_actions),
		num_states_(num_states),
		cnt_explore_(0),
		max_explore_(max_explore),
		learning_rate_(learning_rate),
		discount_factor_(discount_factor),
		learn_(learn),
		q_array_(q_array_init)
{
	size_array_ = num_actions_ * num_states_;
	Init();
	srand ( time(NULL) );
}

void QLearner::Update(double reward, int state, int state_p, int action)
{
	if (state >= num_states_ || state < 0 ||
		state_p >= num_states_ || state_p < 0 ||
		action >= num_actions_ || action < 0)
	{
		ROS_FATAL("Update args out of bounds.");
	}

	// State then action
	int vector_index = state * num_actions_ + action;
	double max_future_val = GetMaxActionQVal(state_p);
	q_array_[vector_index] += learning_rate_ * (reward + discount_factor_ *
			max_future_val - q_array_[vector_index]);
}

int QLearner::GetAction(int state)
{
	int action;

	if (learn_)
	{
		// Check if we should explore
		if (cnt_explore_ > max_explore_)
		{
			cnt_explore_ = 0;
			action = GetRandAction();
		}
		else
		{
			action = GetMaxAction(state);
			cnt_explore_++;
			if (action < 0)
				action = GetRandAction();
		}
	}
	else
	{
		action = GetMaxAction(state);
	}

	return action;
}

void QLearner::Init(void)
{
	if (learning_rate_ > 1 || learning_rate_ <= 0)
		ROS_FATAL("Learning rate is not 0 < lr <= 1.");

	if (discount_factor_ >= 1 || discount_factor_ < 0)
		ROS_FATAL("Discount factor is not 0 <= df < 1.");

	if (size_array_ != (int)q_array_.size())
		ROS_FATAL("Size of array is not the correct size.");
}

int QLearner::GetRandAction(void)
{
	int action = rand() % num_actions_;
	return action;
}

int QLearner::GetMaxAction(int state)
{
	int first_index = state * num_actions_;
	int last_index = first_index + num_actions_;
	int index = first_index;
	double max = q_array_[first_index];

	for (int i = first_index; i < last_index; i++)
	{
		if (q_array_[i] > max)
		{
			max = q_array_[i];
			index = i;
		}
	}

	return index % num_actions_;
}

double QLearner::GetMaxActionQVal(int state)
{
	int first_index = state * num_actions_;
	int last_index = first_index + num_actions_;
	double max = q_array_[first_index];

	for (int i = first_index; i < last_index; i++)
	{
		if (q_array_[i] > max)
		{
			max = q_array_[i];
		}
	}

	return max;
}

std::string QLearner::PrintTable(void)
{
	std::string s;
	for(int state = 0; state < num_states_; state++)
	{
		for(int action = 0; action < num_actions_; action++)
		{
			s.append(boost::lexical_cast<std::string>(q_array_[state * num_actions_ + action]) + ", ");
		}
		s.append("\n");
	}

	return s;
}

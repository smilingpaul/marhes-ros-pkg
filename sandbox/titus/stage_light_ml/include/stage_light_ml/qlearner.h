/*
 * qlearner.h
 *
 *  Created on: Apr 2, 2011
 *      Author: titus
 */

#ifndef QLEARNER_H_
#define QLEARNER_H_

#include "ros/ros.h"
#include <vector>
#include <cstdlib>
#include <time.h>

class QLearner
{
public:
	QLearner(int num_actions, int num_states, double learning_rate,
			double discount_factor, bool learn, int max_explore);
	QLearner(int num_actions, int num_states, double learning_rate,
			double discount_factor, bool learn, int max_explore,
			std::vector<double> q_array_init);
	void Update(double reward, int state, int state_p, int action);
	int GetAction(int state);
	std::string PrintTable(void);

private:
	int num_actions_, num_states_, size_array_, cnt_explore_, max_explore_;
	double learning_rate_, discount_factor_;
	bool learn_;
	std::vector<double> q_array_;

	void Init(void);
	int GetRandAction(void);
	int GetMaxAction(int state);
	double GetMaxActionQVal(int state);
};

#endif /* QLEARNER_H_ */

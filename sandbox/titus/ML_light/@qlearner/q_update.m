function obj = q_update(A, reward, state, state_p, action)

obj = A;
max_future_val = get_max_action_qval(obj, state_p);
obj.qtable(state, action) += obj.learning_rate * (reward + obj.discount_factor * max_future_val - obj.qtable(state, action));
obj.qtable;

end
function v = get_max_action_qval(A, state)

[v, a] = max(A.qtable(state, :));

end
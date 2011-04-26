function a = get_max_action(A, state)

[val, a] = max(A.qtable(state, :));

end
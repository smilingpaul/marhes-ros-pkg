function r = robot(dt, init_state, radius)

A.X(1, 1) = init_state(1, 1);		% Store initial X
A.X(2, 1) = init_state(2, 1);		% Store initial Y
A.X(3, 1) = init_state(3, 1);		% Store initial theta
A.dt = dt;						% Store delta time
A.radius = radius;
r = class(A, 'robot');

end
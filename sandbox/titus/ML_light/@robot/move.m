function r = move(A, vels)

r = A;
t_end = size(r.X, 2);
theta = r.X(3, t_end);
r.X(:, t_end + 1) = r.X(:, t_end) + [cos(theta) 0; sin(theta) 0; 0 1] * r.dt * vels;

end
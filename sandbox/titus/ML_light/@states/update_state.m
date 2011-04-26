function A = update_state(A, pos)

% pos is a vector [x y theta]'
% two light sensors one at fl (0.3m, 0.15m) and one at fr (0.3m, -0.15m)

% Find X, Y of light sensors
pos_flls = pos(1:2) + [cos(pos(3) + A.ang) * A.hyp; sin(pos(3) + A.ang) * A.hyp];
pos_frls = pos(1:2) + [cos(pos(3) - A.ang) * A.hyp; sin(pos(3) - A.ang) * A.hyp];

[val, ix] = min(abs(A.X(1, :) - pos_flls(1)));
[val, iy] = min(abs(A.Y(:, 1) - pos_flls(2)));
val_flls = A.Z(ix, iy);

[val, ix] = min(abs(A.X(1, :) - pos_frls(1)));
[val, iy] = min(abs(A.Y(:, 1) - pos_frls(2)));
val_frls = A.Z(ix, iy);

threshold = 0.1;
ls_diff = val_flls - val_frls;
if ls_diff > threshold
	A.state = 1;
else if ls_diff < -threshold
	A.state = 3;
else
	A.state = 2;
end
	
light_reading = (val_flls + val_frls) / 2;
A.reward = light_reading - A.light_last;
A.light_last = light_reading;

end

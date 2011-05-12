num_reps = 100;
freq = 0.5;
start_state = [-7.5 7.5 2]';
robot_radius = 0.35;
goal_radius = 1.0;
learn = true;
learning_rate = 0.5;
discount_factor = 0.5;
max_explore = 5;

light_origin = [-10 10]';
map_lims = [-20 -0; 20 0];
max_lum = 10;
res = 0.01;
lsns_hyp = 0.3;
lsns_ang = 0.5;

t_sample = 0.1;
t_start = 0;

cnt_rep = 0;
robot_obj = robot(t_sample, start_state, robot_radius);
state_obj = states(light_origin, map_lims, max_lum, res, lsns_hyp, lsns_ang);
action_obj = actions();
qobj = qlearner(get_num_actions(action_obj),
                          get_num_states(state_obj),
                          learning_rate,
                          discount_factor,
                          learn,
                          max_explore);

state_obj = update_state(state_obj, getX(robot_obj));
state = get_state(state_obj);

while cnt_rep < num_reps
	while distance(robot_obj, light_origin) > goal_radius
		[qobj, action] = get_action(qobj, state);
		vels = get_velocities(action_obj, action);
		
		for t = t_start:(t_start + freq / t_sample)
			robot_obj = move(robot_obj, vels);
			robot_plot(robot_obj);
			xlim(map_lims(1, :));
			ylim([map_lims(2, 2) map_lims(2, 1)]);
			filename=sprintf('output/%05d.png',t);
       			print(filename);
		end
		
		t_start = t;
		
		state_obj = update_state(state_obj, getX(robot_obj));
		state_p = get_state(state_obj);
		
		if learn
			X = getX(robot_obj);
			in = inpolygon(X(1, 1), X(2, 1), 
			       [map_lims(1,1) map_lims(1,2) map_lims(1,2) map_lims(1,1)],
			       [map_lims(2,1) map_lims(2,1) map_lims(2,2) map_lims(2,2)]); 
			if in == 0
				reward = -2;
			else
				reward = get_reward(state_obj);
			end
			qobj = q_update(qobj, reward, state, state_p, action);	
		        disp(["t: ", num2str(t), ". From ", num2str(state), " to ", num2str(state_p), ". Action ", num2str(action), " with reward ", num2str(reward)]);
		end
		
		state = state_p;
	end
	
	robot_obj = setX(robot_obj, start_state);
	state_obj = update_state(state_obj, getX(robot_obj));
	state = get_state(state_obj);
	cnt_rep++;
	disp(["Rep: ", num2str(cnt_rep)]);
end

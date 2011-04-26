function [obj, a] = get_action(A, state)

obj = A;

if obj.learn
	if obj.cnt_explore > obj.max_explore
		obj.cnt_explore = 0;
		a = get_rand_action(obj);
	else
		a = get_max_action(obj, state);
		obj.cnt_explore++;
		if a < 0
			a = get_rand_action(obj);
		end
	end
else
	a = get_max_action(obj, state);
end

end
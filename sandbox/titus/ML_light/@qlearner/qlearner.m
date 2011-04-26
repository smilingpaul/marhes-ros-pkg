function q = qlearner(varargin)

% num_actions, num_states, learning_rate, discount_factor, learn, max_explore, q_array_init

switch(nargin)
	case 5				% Q array initialized already
		A.learning_rate = varargin{1};
		A.discount_factor = varargin{2};
		A.learn = varargin{3};
		A.max_explore = varargin{4};
		A.qtable = varargin{5};
		[A.num_states, A.num_actions] = size(A.qtable);
	case 6				% Q array not initialized
		A.num_actions = varargin{1};
		A.num_states = varargin{2};
		A.learning_rate = varargin{3};
		A.discount_factor = varargin{4};
		A.learn = varargin{5};
		A.max_explore = varargin{6};
		A.qtable = 0.2 * rand(A.num_states, A.num_actions);
	otherwise
		error('QLearner number of args not correct.');	
end

A.cnt_explore = 0;

q = class(A, 'qlearner');

end
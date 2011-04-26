function a = actions()

A.action_vels = [0.5 1.0;
                           0.5 0.0;
                           0.5 -1.0]';
                             
A.num_actions = 3;

a = class(A, 'actions');

end
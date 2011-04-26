close all;

r = robot(0.1, [-5 5 -3]', 0.35);

for t = 0:50
	r = move(r, [0.5 1.0]');
	robot_plot(r);
	xlim([-10 0]);
	ylim([0 10]);
	filename=sprintf('output/%05d.png',t);
        print(filename);
	
end
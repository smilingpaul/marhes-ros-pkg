function h = robot_plot(A)

circle(A.X(1, end), A.X(2, end), A.radius);
line(A.X(1, end) + [0 A.radius*1.5*cos(A.X(3, end)) A.radius*1.25*cos(A.X(3, end)-0.25) A.radius*1.25*cos(A.X(3, end)+0.25) A.radius*1.5*cos(A.X(3, end))], 
       A.X(2, end) + [0 A.radius*1.5*sin(A.X(3, end)) A.radius*1.25*sin(A.X(3, end)-0.25) A.radius*1.25*sin(A.X(3, end)+0.25) A.radius*1.5*sin(A.X(3, end))], 
       'Color', 'blue', 'linewidth', 2);

end
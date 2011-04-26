function circle(x, y, radius)

t = 0:0.25:2*pi;
x_t = radius * cos(t) + x;
y_t = radius * sin(t) + y;
plot(x_t, y_t, 'linewidth', 2);

end
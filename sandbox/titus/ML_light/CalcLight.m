%function [X, Y, Z] = CalcLight(x_src, y_src, x_lim, y_lim, res, max_lum)

x_src = -10;
y_src = 10;
x_lim = [-15 -5];
y_lim = [5 15];
res = 0.1;
max_lum = 1000;

x = x_lim(1, 1):res:x_lim(1, 2);
y = y_lim(1, 1):res:y_lim(1, 2);
[X, Y] = meshgrid(x, y);
%Z = 100 * (1 - (sqrt((X - x_src).^2 + (Y - y_src).^2) / sqrt((sum(x_lim) / 2)^2 + (sum(y_lim) / 2)^2)).^2);
Z = max_lum ./ ((sqrt((X - x_src).^2 + (Y - y_src).^2)).^2);
i = Z > max_lum;
Z(i) = max_lum;

surf(X, Y, Z, 'EdgeColor', 'none');
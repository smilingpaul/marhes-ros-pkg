function s = states(light_source, map_lim, max_lum, res, hyp, ang)

A.light_source = light_source;	% [x y]'
A.map_lim = map_lim;			% Array top left corner then lower right corner, [tlx lrx; tly lry]
A.max_lum = max_lum;
A.res = res;
A.hyp = hyp;
A.ang = ang;

x = A.map_lim(1, 1):A.res:A.map_lim(1, 2);
y = A.map_lim(2, 2):A.res:A.map_lim(2, 1);
[A.X, A.Y] = meshgrid(x, y);
A.Z = A.max_lum ./ ((sqrt((A.X - A.light_source(1, 1)).^2 + (A.Y - A.light_source(2, 1)).^2)).^2);
i = A.Z > A.max_lum;
A.Z(i) = A.max_lum;

A.light_last = 0;
A.state = 0;
A.reward = 0;
A.num_states = 3;
surf(A.X, A.Y, A.Z);
s = class(A, 'states');

end
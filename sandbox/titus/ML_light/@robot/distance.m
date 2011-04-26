function d = distance(A, coords)

d = sqrt(sum((A.X(1:2, end) - coords).^2));

end
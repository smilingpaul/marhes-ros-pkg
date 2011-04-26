function r = setX(A, state)

r = A;
r.X(:, end + 1) = state;

end
function dq = cayley_map(dtheta)
% dtheta must be 3x1
mag = 1/(sqrt(1+dtheta'*dtheta));

dq = mag*[1;dtheta];

end
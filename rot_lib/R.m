function R_matrix = R(q)
    % Right-multiply
    
    % Extract the quaternion elements
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % Define the identity matrix
    I = eye(3);
    
    % Create the R matrix
    R_matrix = zeros(4, 4);
    R_matrix(1, 1) = w;
    R_matrix(1, 2:end) = -[x, y, z];
    R_matrix(2:end, 1) = [x, y, z];
    R_matrix(2:end, 2:end) = w * I - skew([x, y, z]);
end
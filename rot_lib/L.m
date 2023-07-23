function L_matrix = L(q)
    % Left-multiply
    
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    

    I = eye(3);
    
    L_matrix = zeros(4, 4);
    L_matrix(1, 1) = w;
    L_matrix(1, 2:end) = -[x, y, z];
    L_matrix(2:end, 1) = [x, y, z];
    L_matrix(2:end, 2:end) = w * I + skew([x, y, z]);
end
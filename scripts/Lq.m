  function L_matrix = Lq(q_local)
    % Left-multiply

    w = q_local(1);
    x = q_local(2);
    y = q_local(3);
    z = q_local(4);

    I = eye(3);

    L_matrix = zeros(4, 4,'like',q_local); % Initialize a 4x4 

    L_matrix(1, 1) = w;
    L_matrix(1, 2:end) = -[x, y, z];
    L_matrix(2:end, 1) = [x; y; z]; 
    L_matrix(2:end, 2:end) = w * I + skew([x, y, z]);
  end
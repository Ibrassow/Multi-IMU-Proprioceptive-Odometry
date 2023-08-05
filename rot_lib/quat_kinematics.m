function q_dot = quat_kinematics(q, omega)
    % q: (w, x, y, z)
    q_dot = 0.5 * quat_multiplication(q, [0;omega]');
    %omega_mat = @(w) [0, -w(1), -w(2), -w(3);
     %          w(1), 0, w(3), -w(2);
      %         w(2), -w(3), 0, w(1);
       %        w(3), w(2), -w(1), 0];
    %q_dot = 0.5 * omega_mat(omega) * q';

end
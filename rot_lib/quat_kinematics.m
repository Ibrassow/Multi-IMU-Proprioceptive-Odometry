function q_dot = quat_kinematics(q, omega)
    % q: (w, x, y, z)
    q_dot = 0.5 * quat_multiplication(q, [0;omega]');

end
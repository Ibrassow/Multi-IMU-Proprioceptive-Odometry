


q = [0.7493,0.558,0.4683,0];
q = q / norm(q);


om = [0.2 0.2 0.3]';
ext_om = [0; om];

q1 = quat_kinematics(q, om);
q2 = 0.5 * quat_multiplication(q, ext_om);

u1 = expm(R(ext_om))
u2 = expm(mtx_w_to_quat_dot(om))




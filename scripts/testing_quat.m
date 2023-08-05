
nf = 500;
%mm = zeros(4,nf);

for i=1:nf
    %mm(:,i) = euler_to_quat(mipo_state_list(7:9,i));

end

omega_mat = @(w) [0, -w(1), -w(2), -w(3);
               w(1), 0, w(3), -w(2);
               w(2), -w(3), 0, w(1);
               w(3), w(2), -w(1), 0];


%plot(miqpo_state_list(7:10,1:nf)')
%hold on
%plot(mipo_state_list(7:9,1:nf)')
%plot(mm')


qq = [0.98, 0.1, 0.065, 0.5];
qq = qq / norm(qq);

w = [0.25, 0.1, -0.170];

H = [zeros(1, 3); eye(3)];

G = @(q) L(q) * H; 

qqd1 = quat_kinematics(qq, w')
qqd2 = 0.5 * L(qq) * H * w'
qqd3 = 0.5 * L(qq) * [0,w]'
qqd4 = 0.5 * G(qq) * w'
qqd5 = 0.5 * omega_mat(w) * qq'

qqq = [0.0, -0.1, 0.05, -0.5];
qqq = qqq / norm(qqq);

res1 = L(qq) * qqq'
res2 = quat_multiplication(qq, qqq)






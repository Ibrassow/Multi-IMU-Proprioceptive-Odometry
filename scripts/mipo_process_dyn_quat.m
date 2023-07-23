function xdot = mipo_process_dyn_quat(x,u)

% the process dynamics of attitude-lo-foot filter
% but with additional IMUs on feet
% state x 
%   - position          (1:3)
%   - velocity          (4:6)
%   - quaternion        (7:10)
%   - foot1 pos         (11:13)
%   - foot1 vel         (14:16)
%   - foot2 pos         (17:19)
%   - foot2 vel         (20:22)
%   - foot3 pos         (23:25)
%   - foot3 vel         (26:28)
%   - foot4 pos         (29:31)
%   - foot4 vel         (32:34)
%   - body acc bias     (35:37)
%   - body gyro bias    (38:40)
%   - foot1 acc bias    (41:43)
%   - foot2 acc bias    (44:46)
%   - foot3 acc bias    (47:49)
%   - foot4 acc bias    (50:52)
%   - time  tk          (53)

% control u 
%   - w      (1:3)      body IMU angular veolocity
%   - a       (4:6)     body IMU acceleration
%   - a1      (7:9)     foot 1 IMU acceleration (already in body frame)
%   - a2      (10:12)   foot 2 IMU acceleration (already in body frame)
%   - a3      (13:15)   foot 3 IMU acceleration (already in body frame)
%   - a4      (16:18)   foot 4 IMU acceleration (already in body frame)
%   - hk       (19)

% dot x 
%   - velocity    
%   - acc    
%   - dquaternion 
%   - foot1 vel 
%   - foot1 acc 
%   - foot2 vel 
%   - foot2 acc 
%   - foot3 vel 
%   - foot3 acc 
%   - foot4 vel 
%   - foot4 acc 
%   - hk   

if ~iscolumn(x)
    x = x';
end

if ~iscolumn(u)
    u = u';
end

pos = x(1:3);
vel = x(4:6);
quat = x(7:10);

foot1_pos = x(11:13);
foot1_vel = x(14:16);
foot2_pos = x(17:19);
foot2_vel = x(20:22);
foot3_pos = x(23:25);
foot3_vel = x(26:28);
foot4_pos = x(29:31);
foot4_vel = x(32:34);

ba = x(35:37);     % body acc bias
bg = x(38:40);     % gyro bias

foot1_ba = x(41:43);     % foot 1 acc bias
foot2_ba = x(44:46);     % foot 2 acc bias
foot3_ba = x(47:49);     % foot 3 acc bias
foot4_ba = x(50:52);     % foot 4 acc bias

w = u(1:3) - bg;     % body angular velocity
a = u(4:6) - ba;    % body linear acceleration


foot1_acc = u(7:9)   - foot1_ba;  % already changed to body frame
foot2_acc = u(10:12) - foot2_ba;  % already changed to body frame
foot3_acc = u(13:15) - foot3_ba;  % already changed to body frame
foot4_acc = u(16:18) - foot4_ba;  % already changed to body frame


dquat = quat_kinematics(quat, w);

R = quat_to_rot(quat);

acc = R*a -[0;0;9.8];

foot1_acc_w = R*foot1_acc - [0;0;9.8];
foot2_acc_w = R*foot2_acc - [0;0;9.8];
foot3_acc_w = R*foot3_acc - [0;0;9.8];
foot4_acc_w = R*foot4_acc - [0;0;9.8];

xdot = [vel;acc;dquat;
        foot1_vel;foot1_acc_w;
        foot2_vel;foot2_acc_w;
        foot3_vel;foot3_acc_w;
        foot4_vel;foot4_acc_w;
        zeros(3,1);                  % body acc bias
        zeros(3,1);                  % gyro bias
        zeros(3,1);                  % foot 1 acc bias
        zeros(3,1);                  % foot 2 acc bias
        zeros(3,1);                  % foot 3 acc bias
        zeros(3,1);                  % foot 4 acc bias
        1];  
end
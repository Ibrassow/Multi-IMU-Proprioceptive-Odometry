function [kf_conf,param] = mipo_conf_init_quat(param)
import casadi.*

% Use casadi + "Planning with Attitude" paper to compute jacobians

kf_conf = {};

kf_conf.state_size = 53;   
kf_conf.error_state_size = kf_conf.state_size - 1;
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

temp_fix_yaw = 1;


param.mipo_meas_per_leg = 11;  % 3 3 3 1 
kf_conf.meas_size = param.mipo_meas_per_leg * param.num_leg + 1; % yaw
param.mipo_meas_size = kf_conf.meas_size + temp_fix_yaw;

kf_conf.ctrl_size = 19;  
% control u 
%   - w      (1:3)      body IMU angular veolocity
%   - a       (4:6)     body IMU acceleration
%   - a1      (7:9)     foot 1 IMU acceleration (already in body frame)
%   - a2      (10:12)   foot 2 IMU acceleration (already in body frame)
%   - a3      (13:15)   foot 3 IMU acceleration (already in body frame)
%   - a4      (16:18)   foot 4 IMU acceleration (already in body frame)
%   - hk       (19)


% process noise
kf_conf.Q1 = diag([1e-4*ones(3,1); 
           1e-2*ones(3,1); 
           1e-5*ones(3,1);
           repmat(...
           [1e-4*ones(2,1);  % foot1 pos x y
            1e-4*ones(1,1);  % foot1 pos z
            1e-4*ones(2,1);  % foot1 vel  x y
            1e-4*ones(1,1)],4,1);  % foot1 vel z
           1e-4*ones(3,1);    % acc bias random walk
           1e-4*ones(3,1);    % gyro bias random walk
           repmat(...
           [1e-4*ones(3,1)],4,1);  % foot acc bias
           0;]);                  % do not use tk in KF

% control noise 
kf_conf.Q2 = diag([1e-4*ones(3,1);   % BODY acc noise
                   1e-4*ones(3,1);   % gyro noise
                   repmat(...
           [1e-4*ones(3,1)],4,1);    % foot acc noise
                   0]);

kf_conf.R = 1e-2*eye(kf_conf.meas_size+temp_fix_yaw);


% get process and process jacobians
s_xk = casadi.MX.sym('X_k', kf_conf.state_size);
s_uk = casadi.MX.sym('Uk', kf_conf.ctrl_size);
s_uk1 = casadi.MX.sym('Uk1', kf_conf.ctrl_size);
s_dt = casadi.MX.sym('dt', 1);
s_f = dyn_rk4_quat(s_xk , s_uk, s_uk1, s_dt, @mipo_process_dyn_quat); % EKF process


%% TODO -- 
H = [MX.zeros(1, 3); MX.eye(3)];
G = @(q) Lq(q) * H; 

% at k
Ek = casadi.MX(zeros(53, 52));
Ek(1:6,1:6) = eye(6);
Ek(7:10, 7:9) = G(s_xk(7:10)); 
Ek(11:53, 10:52) = eye(43);


% at k+1
Ek1 = casadi.MX(zeros(53, 52));
s_xk1 = dyn_rk4_quat(s_xk, s_uk1, s_uk1, s_dt, @mipo_process_dyn_quat);
Ek1(1:6,1:6) = eye(6);
Ek1(7:10, 7:9) = G(s_xk1(7:10)); 
Ek1(11:53, 10:52) = eye(43);



s_F = Ek1' * jacobian(s_f, s_xk) * Ek;
s_B = Ek1' * jacobian(s_f, s_uk);


% Function for generating the process dynamics and jacobians
kf_conf.f = Function('process',{s_xk, s_uk, s_uk1, s_dt},{s_f});
kf_conf.df = Function('process_jac',{s_xk, s_uk, s_uk1, s_dt},{s_F});
kf_conf.db = Function('control_jac',{s_xk, s_uk, s_uk1, s_dt},{s_B});



% get measurement jacobians
s_phik = casadi.MX.sym('phik', 12);
s_wk = casadi.MX.sym('wk', 3);
s_dphik = casadi.MX.sym('dphik', 12);
s_yawk = casadi.MX.sym('yaw', 1);
s_foot_gyrok = casadi.MX.sym('foot_gyro', 12);
s_r = mipo_measurement_quat(s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok, param);



Ekm = casadi.MX(zeros(53, 52));
Ekm(1:6,1:6) = eye(6);
Ekm(7:10, 7:9) = G(s_xk(7:10)); 
Ekm(11:53, 10:52) = eye(43);


s_R = jacobian(s_r, s_xk) * Ekm; 
% yaw ? 

kf_conf.r = Function('meas',{s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok},{s_r});
kf_conf.dr = Function('meas_jac',{s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok},{s_R});



% foot IMU frame has a transformation wrt foot center 
% FL FR RL RR
param.R_fm_list = {[-1  0  0;
               0  0 -1;
               0 -1  0], 
             [-1   0   0; 
               0   0   1;
               0   1   0],
             [-1  0  0;
               0  0 -1;
               0 -1  0],
             [-1   0  0; 
               0   0  1;
               0   1  0]};
end
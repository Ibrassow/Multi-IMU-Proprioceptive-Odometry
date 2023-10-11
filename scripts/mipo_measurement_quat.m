function meas_residual = mipo_measurement_quat(x, wk, phik, dphik, yawk, foot_gyrok, param)

% this measurement function calculates mipo measurement

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

    if ~iscolumn(x)
        x = x';
    end


    pos = x(1:3);
    vel = x(4:6);
    quat = x(7:10);
    R_er = quat_to_rot(quat);

    
    foot_pos = [x(11:13); x(17:19); x(23:25); x(29:31)];
    foot_vel = [x(14:16); x(20:22); x(26:28); x(32:34)];
    bg = x(38:40);

    meas_per_leg = param.mipo_meas_per_leg;
    meas_residual = zeros(param.mipo_meas_size,1,class(x));



    for i = 1:param.num_leg
        % M
        angle = phik((i-1)*3+1:(i-1)*3+3);
        av = dphik((i-1)*3+1:(i-1)*3+3);
        p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        J_rf = autoFunc_d_fk_dt(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
        leg_v = (J_rf*av+skew(wk-bg)*p_rf);


        % Measurement residual top 
        meas_residual((i-1)*meas_per_leg+1:(i-1)*meas_per_leg+3) = ...
            p_rf - R_er'*(foot_pos((i-1)*3+1:(i-1)*3+3) - pos);
        meas_residual((i-1)*meas_per_leg+4:(i-1)*meas_per_leg+6) = ...
            foot_vel((i-1)*3+1:(i-1)*3+3) - (vel + R_er*leg_v);


        % use foot gyro to calculate a velocity
        foot_w_robot = foot_gyrok((i-1)*3+1:(i-1)*3+3);
        foot_w_world = R_er*foot_w_robot;
        p_rf_world = R_er*p_rf;   % the vector pointing from body to foot fl in world frame

        foot_support_vec = -p_rf_world/norm(p_rf_world)*0.05;
        foot_vel_world = cross(foot_w_world, foot_support_vec);

        if (param.mipo_use_foot_ang_contact_model == 1)
            meas_residual((i-1)*meas_per_leg+7:(i-1)*meas_per_leg+9) = ...
                foot_vel((i-1)*3+1:(i-1)*3+3)-foot_vel_world;
        else 
            meas_residual((i-1)*meas_per_leg+7:(i-1)*meas_per_leg+9) = ...
                foot_vel((i-1)*3+1:(i-1)*3+3);
        end
        
        % foot height should be 0 
        meas_residual((i-1)*meas_per_leg+10) = foot_pos((i-1)*3+3);   % assume ground always has 0 height 
    end

    % Yaw comes from the mocap data, euler is euler

    % euler = quat_to_euler(quat);

    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);
    cos_pitch_cos_yaw  = 1.0- 2.0 * (y*y + z*z);   
	cos_pitch_sin_yaw  =      + 2.0 * (x*y + w*z);   
    euler_yaw = atan2(cos_pitch_sin_yaw, cos_pitch_cos_yaw);

   
    meas_residual(end) = yawk - euler_yaw;



end
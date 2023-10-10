function re_sensor_data = resample_sensor_data_quat(sensor_data, dt, param)

end_time_list = [sensor_data.accel_body_IMU.Time(end);
                 sensor_data.gyro_body_IMU.Time(end);
                 sensor_data.accel_fl_IMU.Time(end);
                 sensor_data.gyro_fl_IMU.Time(end);
                 sensor_data.accel_fr_IMU.Time(end);
                 sensor_data.gyro_fr_IMU.Time(end);
                 sensor_data.accel_rl_IMU.Time(end);
                 sensor_data.gyro_rl_IMU.Time(end);
                 sensor_data.accel_rr_IMU.Time(end);
                 sensor_data.gyro_rr_IMU.Time(end);
                 sensor_data.contact_mode.Time(end)];

min_end_time = min(end_time_list);

re_sensor_data = {};

re_sensor_data.accel_body_IMU = resample(sensor_data.accel_body_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_body_IMU  = resample(sensor_data.gyro_body_IMU, 0:dt:min_end_time);
re_sensor_data.accel_fl_IMU   = resample(sensor_data.accel_fl_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_fl_IMU    = resample(sensor_data.gyro_fl_IMU, 0:dt:min_end_time);
re_sensor_data.accel_fr_IMU   = resample(sensor_data.accel_fr_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_fr_IMU    = resample(sensor_data.gyro_fr_IMU, 0:dt:min_end_time);
re_sensor_data.accel_rl_IMU   = resample(sensor_data.accel_rl_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_rl_IMU    = resample(sensor_data.gyro_rl_IMU, 0:dt:min_end_time);
re_sensor_data.accel_rr_IMU   = resample(sensor_data.accel_rr_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_rr_IMU    = resample(sensor_data.gyro_rr_IMU, 0:dt:min_end_time);
re_sensor_data.foot_force     = resample(sensor_data.foot_force, 0:dt:min_end_time);
re_sensor_data.contact_mode   = resample(sensor_data.contact_mode, 0:dt:min_end_time);
re_sensor_data.joint_torque   = resample(sensor_data.joint_torque, 0:dt:min_end_time);
re_sensor_data.joint_ang      = resample(sensor_data.joint_ang, 0:dt:min_end_time);
re_sensor_data.joint_vel      = resample(sensor_data.joint_vel, 0:dt:min_end_time);

if param.has_mocap == 1
    re_sensor_data.orient_mocap   = resample(sensor_data.orient_mocap, 0:dt:min_end_time);
    re_sensor_data.pos_mocap      = resample(sensor_data.pos_mocap, 0:dt:min_end_time);
end

end
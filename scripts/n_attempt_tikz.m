
%%
plot_start=1;
%plot_end = size(sipo_state_list(end,:),2);
plot_end = size(miqpo_state_list(end,:),2);
figure(69);clf;

if param.has_mocap == 1
    plot3(re_sensor_data.pos_mocap.Data(1:end,1),re_sensor_data.pos_mocap.Data(1:end,2),re_sensor_data.pos_mocap.Data(1:end,3), 'LineWidth',1.3)
    hold on;
end


plot3(movmean(sipo_state_list(1,plot_start:plot_end),5,1), ...
      movmean(sipo_state_list(2,plot_start:plot_end),5,1), ...
      movmean(sipo_state_list(3,plot_start:plot_end),5,1), 'LineWidth',1.3);


hold on;

plot3(mipo_state_list(1,plot_start:plot_end),mipo_state_list(2,plot_start:plot_end),mipo_state_list(3,plot_start:plot_end), 'LineWidth',1.3);

hold on;

plot3(miqpo_state_list(1,plot_start:plot_end),miqpo_state_list(2,plot_start:plot_end),miqpo_state_list(3,plot_start:plot_end), 'LineWidth',1.3);

legend("Ground truth", "SIPO","Multi-IMU PO (euler)", "Multi-IMU PO (quaternion)", "Location","northeast")
%legend("Ground truth", "MIQPO", "Location","northeast")
axis equal

xlabel("X Position (m)")
ylabel("Y Position (m)")
view(-0,90)
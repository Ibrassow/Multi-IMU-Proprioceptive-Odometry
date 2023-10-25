
%%
plot_start=1;
%plot_end = size(sipo_state_list(end,:),2);
plot_end = size(miqpo_state_list(end,:),2);
figure(69);clf;

lw = 1.5; %linewidth

if param.has_mocap == 1
    plot3(re_sensor_data.pos_mocap.Data(1:3:end,1),re_sensor_data.pos_mocap.Data(1:3:end,2),re_sensor_data.pos_mocap.Data(1:3:end,3), 'LineWidth',lw)
    hold on;
end


plot3(movmean(sipo_state_list(1,plot_start:3:plot_end),5,1), ...
      movmean(sipo_state_list(2,plot_start:3:plot_end),5,1), ...
      movmean(sipo_state_list(3,plot_start:3:plot_end),5,1), 'LineWidth',lw);


hold on;

plot3(mipo_state_list(1,plot_start:3:plot_end),mipo_state_list(2,plot_start:3:plot_end),mipo_state_list(3,plot_start:3:plot_end), 'LineWidth',lw);

hold on;

plot3(miqpo_state_list(1,plot_start:3:plot_end),miqpo_state_list(2,plot_start:3:plot_end),miqpo_state_list(3,plot_start:3:plot_end), 'LineWidth',lw);

legend("Ground truth", "SIPO","Multi-IMU PO (euler)", "Multi-IMU PO (quaternion)", "Location","northeast")
%legend("Ground truth", "MIQPO", "Location","northeast")
axis equal

xlabel("X Position (m)")
ylabel("Y Position (m)")
view(-0,90)
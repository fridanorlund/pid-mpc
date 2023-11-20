noise = 1;

MPC_active=1;
mpc_controller.W.MV = 1;
mpc_controller.W.OV = 0;
out1 = sim('single_tank_linear_real_PID.slx',sim_time);
mpc_controller.W.OV = 2/3;
mpc_controller.W.MV = 1/3;
out2 = sim('single_tank_linear_real_PID.slx',sim_time);
mpc_controller.W.OV = 0.9;
mpc_controller.W.MV = 0.1;
out3 = sim('single_tank_linear_real_PID.slx',sim_time);
MPC_active = 0;
out_PI = sim('single_tank_linear_real_PID.slx',sim_time);
MPC_active =1;

%% plot results
limits = [400 1000]; % [400 1500]; %

figure
subplot(4,1,1)
yline([ OV_max+h_0])
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
MPC1 = plot(out1.tout(400:end),out1.h(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2);
MPC2 = plot(out2.tout(400:end),out2.h(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
MPC3 = plot(out3.tout(400:end),out3.h(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2);
MPC_PI = plot(out_PI.tout(400:end),out_PI.h(400:end),'color', [0.255 0 0.255], 'LineWidth', 2);
title('Level')
legend_elements = [MPC_PI MPC1 MPC2 MPC3];
legend(legend_elements,'PI','MPC1','MPC2','MPC3','location','northeast')
xlim(limits)


subplot(4,1,2)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1.tout(400:end),out1.u(400:end)-u_0,'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2.tout(400:end),out2.u(400:end)-u_0,'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3.tout(400:end),out3.u(400:end)-u_0,'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
plot(out_PI.tout(400:end),out_PI.u(400:end)-u_0,'color', [0.255 0 0.255], 'LineWidth', 2);
title('Total control signal')
xlim(limits)


subplot(4,1,3)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1.tout(400:end),out1.v(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2.tout(400:end),out2.v(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3.tout(400:end),out3.v(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
plot(out_PI.tout(400:end),out_PI.v(400:end),'color', [0.255 0 0.255], 'LineWidth', 2);
title('PI control signal')
xlim(limits)


subplot(4,1,4)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1.tout(400:end),out1.w(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2.tout(400:end),out2.w(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3.tout(400:end),out3.w(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
title('MPC control signal')
xlim(limits)

sgtitle('Simulation with noise')

close all
clear all
%% Simulation time [s]
dt = 1;                             % Sampling time
sim_time = 1500;                    % Simulation time
t_vector = [1:dt:sim_time]';        % Tector containing timesteps

% creates the statespace models for  controller, filter and process
run('state_space_models.m')

w_ov = 1;       % weight outputs, y
w_mv = 1;       % weight control signal, w
delta = 1;      % modelerror os controlsignal (1 = perfect model)
[mpc_controller, OV_min, OV_max] = MPC_setup(sys_pd,sys_kd,nu,nr,w_ov,w_mv, dt, delta);


%% Parameters for simulation

hdiff = 100;                                        % physical level difference between adjecent cells [cm]
R = 600/2;                                          % radius of tank in upper section [cm]
Acm = pi*R^2;                                       % Crossection area of tank [cm2]

% Valve parameters
K_valve_quad_4010 =  -0.55*100^3/(60*60);
K_valve_lin_4010 = 108*100^3/(60*60);

% noise:
dist.denom = [1 -2.275 1.752 -0.473];       % Denominator koefficients to filter
K_noise = 0*196*10^(-4);                      % Scaling the output to match the model, skaled with the mean of the outputs.



%% Inflow of slurry

% Probably make this smoother sice we are running without buffer.
I = 1.1*100^3;
inflow = [t_vector, 0*ones(length(t_vector),1)]; % deviations from nominal flow goes into the model

% Make an abrupt inflow disturbance, step to half hthe flow:
inflow(round(length(t_vector)/3):2*round(length(t_vector)/3),2) = -I*0.25;

% Milling-line stop 100% till 50% på 5 min:
x = [1 3000 3300 6000 6300 10000];
v = [0 0 -I*0.5 -I*0.5 0 0];
%inflow(:,2) = interpn(x,v,1:10000);


% ocilationg inflow disturbance:
%inflow(:,2) = ( I/5*sin(0.01*t_vector)) ;

%figure
%plot(inflow(:,2))


%% reference value:

ref = 31;
ref = [t_vector, ref*ones(length(t_vector),1)];


%% Run simulation:
MPC_active=1;
mpc_controller.W.MV = 1;
mpc_controller.W.OV = 0;
out1 = sim('single_tank_linear_known_disturbance_real_PID.slx',sim_time);
mpc_controller.W.OV = 2;
out2 = sim('single_tank_linear_known_disturbance_real_PID.slx',sim_time);
mpc_controller.W.OV = 0.9;
mpc_controller.W.MV = 0.1;
out3 = sim('single_tank_linear_known_disturbance_real_PID.slx',sim_time);
MPC_active = 0;
out_PI = sim('single_tank_linear_known_disturbance_real_PID.slx',sim_time);
MPC_active =1;


%% Figure for main results
limits = [400 1000]; %[400 1500];

figure
subplot(4,1,1)
yline([ OV_max+h_0])
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
MPC1 = plot(out1(1,1).tout(400:end),out1.h(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2);
MPC2 = plot(out2(1,1).tout(400:end),out2.h(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
MPC3 = plot(out3(1,1).tout(400:end),out3.h(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2);
MPC_PI = plot(out_PI(1,1).tout(400:end),out_PI.h(400:end),'color', [0.255 0 0.255], 'LineWidth', 2);
title('Level')
legend_elements = [MPC_PI MPC1 MPC2 MPC3];
legend(legend_elements,'PI','MPC1','MPC2','MPC3','location','northeast')
xlim(limits)


subplot(4,1,2)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1(1,1).tout(400:end),out1.u(400:end)-u_0,'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2(1,1).tout(400:end),out2.u(400:end)-u_0,'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3(1,1).tout(400:end),out3.u(400:end)-u_0,'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
plot(out_PI(1,1).tout(400:end),out_PI.u(400:end)-u_0,'color', [0.255 0 0.255], 'LineWidth', 2);
title('Total control signal')
xlim(limits)


subplot(4,1,3)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1(1,1).tout(400:end),out1.v(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2(1,1).tout(400:end),out2.v(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3(1,1).tout(400:end),out3.v(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
plot(out_PI(1,1).tout(400:end),out_PI.v(400:end),'color', [0.255 0 0.255], 'LineWidth', 2);
title('PI control signal')
xlim(limits)


subplot(4,1,4)
xline([round(length(t_vector)/3) 2*round(length(t_vector)/3)])
hold on
plot(out1(1,1).tout(400:end),out1.w(400:end),'color', [0 0.4470 0.7410], 'LineWidth', 2)
plot(out2(1,1).tout(400:end),out2.w(400:end),'color', [0.8500 0.3250 0.0980], 'LineWidth', 2)
plot(out3(1,1).tout(400:end),out3.w(400:end),'color', [0.4660 0.6740 0.1880], 'LineWidth', 2)
title('MPC control signal')
xlim(limits)

sgtitle('Simulation with known inflow disturbance')
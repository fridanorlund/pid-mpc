
close all
clear all
%% Simulation time [s]
dt = 1;                             % Sampling time
sim_time = 1500;                    % Simulation time
t_vector = [1:dt:sim_time]';        % Tector containing timesteps

% Set up the state-space mopdels for controller, plant and closed loop.
run('state_space_models.m');
% Set up needed variables to run simulation
run('simulation_setup.m');

% Set up the MPC controller
w_ov = 1;       % weight outputs, y
w_mv = 1;       % weight control signal, w
delta = 1;      % modelerror on controlsignal gain (1 = perfect model)
[mpc_controller, OV_min, OV_max] = MPC_setup(sys_pd,sys_kd,nu,nr,w_ov,w_mv, dt, delta);



%% Run simulations:

% Simulation without noise
run('run_simulation_smooth.m');
% Simulation with noise
run('run_simulation_noise.m');
% Simulation with model errors
run('run_simulation_model_errors.m')

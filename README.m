% This folder simulates the PID+MPC control structure for a single
% flotation cell (linear model) to demonstrate the control structure.

%% Folder content:
% This folder contains the folowing files:

% - linear_version_real_PID.m
% - MPC_setup.m
% - run_simulation_model_errors.m
% - run_simulation_noise.m
% - run_simulation_smooth.m
% - simulation_setup.m
% - state_space_models.m

% - single_tank_linear_real_PID.slx


%% Detailed descripion

% - linear_version_real_PID.m:
%    This is the "main" file in this folder. This file runs the other files
%    as part of the setup. To reproduce the results in the paper, simply
%    run this file.

% - MPC_setup.m
%    This function sets up the MPC controller with the given weight
%    matrices and system models. In the control design, the model errors
%    arealso introduced.

% - run_simulation_model_errors.m
%    This file runs the simulations for 3 different MPC-tunings and for
%    each tuning, 3 different versions of model errors.

% - run_simulation_noise.m
%    This file runs the simulation for the given MPC-tunings with noise
%    added to the measurement.

% - run_simulation_smooth.m
%    This file runs the simulation for the given MPC-tunings without noise
%    added to the measurement.

% - simulation_setup.m
%    This file contains all parameters needed for the simulation to run,
%    such as reference values, the inflow disturbance and so on.

% - state_space_models.m
%    This file contains the state space models for the controller and the
%    plant



% - single_tank_linear_real_PID.slx
%    This is the simuling model used to simulate.
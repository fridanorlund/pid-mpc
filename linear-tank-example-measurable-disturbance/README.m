% This folder contains the code for runing the simulation of the PID+MPC
% controlstructure controling a single flotation cell, under the assumption
% that the inflow disturbance is known.

%% Folder content:

% - linear_version_known_disturbance_real_PID.m
% - MPC_setup.m
% - state_space_models.m

% - single_tank_linear_known_disturbance_real_PID.slx


%% Detailed description

% - linear_version_known_disturbance_real_PID.m
%    This is the "main" file in this folder. This file runs the other files
%    as part of the setup. To reproduce the results in the paper, simply
%    run this file. 

% - MPC_setup.m
%    This function sets up the MPC controller with the given weight
%    matrices and system models. 

% - state_space_models.m
%    This file contains the state space models for the controller and the
%    plant



% - single_tank_linear_known_disturbance_real_PID.slx
%    This is the simuling model used to simulate.
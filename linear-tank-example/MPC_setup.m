function [mpc_controller, OV_min, OV_max] = MPC_setup(sys_pd,sys_kd, nu, nr, w_ov, w_mv, dt, delta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% Set up for the MPC:

% Introduce model errors: delta = 1 is no model errors, +- some decimals
% alters the "outflow dynamic or the internal model in the MPC"


% Process model
Ap = sys_pd.A;
Bp = delta*sys_pd.B(:,1:nu);
Cp = sys_pd.C;
Dp = sys_pd.D(:,1:nu);

% Controler model
Ak = sys_kd.A;
Bkr = sys_kd.B(:,1:nr);
Bky = sys_kd.B(:,nr+1:end);
Ck = sys_kd.C;
Dkr = sys_kd.D(:,1:nr);
Dky = sys_kd.D(nr+1:end);


% Helper equations
Ep = inv( eye(length(Dky(:,1))) - Dky*Dp );
Ek = inv( eye(length(Dp(:,1))) - Dp*Dky );


% Closed loop matrices
A = [ Ap + Bp*Ep*Dky*Cp   Bp*Ep*Ck
      Bky*Ek*Cp       Ak + Bky*Ek*Dp*Ck ];

Br = [ Bp*Ep*Dkr
      Bkr + Bky*Ek*Dp*Dkr];

Bw = [ Bp + Bp*Ep*Dky*Dp
        Bky*Ek*Dp];

C = [ Ek*Cp   Ek*Dp*Ck ];

Dr = Ek*Dp*Dkr;

Dw = Ek*Dp;

% state space:
% (In discrete time)
sys_d = ss(A,[Br Bw], C, [Dr Dw],dt);
pole(sys_d);
%figure
%bode(sys_d,o)

%% Seting up MPC-controller:

%dimensions of r and w
nw = length(Bw(1,:));
nr = length(Br(1,:));
nwr = nw+nr;

% define which inputs are MV:s and Which are "disturbances" (placing r here)
sys_d = setmpcsignals(sys_d, 'MD',1:nr, 'MV', nr+1:nwr);

% create controller object 
%mpc_controller = mpc(sys_d, dt, 30, 15);
mpc_controller = mpc(sys_d, dt, 150, 50);

% put constraints on MV, OP
MV_max = 30;
MV_min = -70;
MV_rate_max = 0.5;
MV_rate_min = -0.5;
OV_max = 10;
OV_min = -10;
mpc_controller.MV = struct('Min',MV_min,'Max',MV_max);
mpc_controller.OV = struct('Min',OV_min,'Max',OV_max);

% specify weight-matrices for minimization:
mpc_controller.W.MV = w_mv;
mpc_controller.W.ManipulatedVariablesRate = 0;
mpc_controller.W.OV = w_ov;
end
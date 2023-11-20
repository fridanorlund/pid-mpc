%% PID with 2:and order filter on state space form:
% 
% Parameters in PID controller
K = 0.9;
Ti = 87;
Td = 0;
b = 1;
% filter parameters
w = (2*pi/Ti)*100;
z = 0.7071;

s = tf('s');
G_filter = w^2/(s^2 +2*z*w*s + w^2);
pole(G_filter);
o = bodeoptions;
o.MagUnits = 'abs';
%bode(G_filter,o)

% define state
A_kc = [ 0 -1     0 
         0  0     1
         0 -w^2 -2*z*w];

B_kc = [ 1 0
         0 0 
         0 w^2];

C_kc = [K/Ti -K -K*Td];

D_kc = [K*b 0];


sys_kd = c2d(ss(A_kc,B_kc,C_kc,D_kc), dt);

pole(sys_kd);

% controller dimensions:
nu = length(C_kc(:,1));
nr = 1;

% Initial state
I_0 = [ 0 0 0 ];


%% Statespace for single cell:

h_0 = 30;               % froth thickness in cm
u_0 = 60;               % controlsignal in % [0 100]
Acm = (pi*300^2);       % Crossection area of cell in cm2

A_pc = -0.0218101218311116;
B_pc = 0.0520692097769781;
B_pc_d = -(1/Acm);
C_pc = 1;
D_pc = 0;

% diskretisera
sys_pd = c2d(ss(A_pc,[B_pc B_pc_d],C_pc,D_pc), dt);

% Initial state (dh)
x_0 = 0;

%% Parameters for simulation

hdiff = 100;                                        % physical level difference between adjecent cells [cm]
R = 600/2;                                          % radius of tank in upper section [cm]
Acm = pi*R^2;                                       % Crossection area of tank [cm2]

% Valve parameters
K_valve_quad_4010 =  -0.55*100^3/(60*60);
K_valve_lin_4010 = 108*100^3/(60*60);

% noise:
dist.denom = [1 -2.275 1.752 -0.473];       % Denominator koefficients to filter
K_noise = 196*10^(-4);                      % Scaling the output to match the model, skaled with the mean of the outputs.
K_white = 1;

%% Inflow of slurry

% Probably make this smoother sice we are running without buffer.
I = 1.1*100^3;
inflow = [t_vector, 0*ones(length(t_vector),1)];

% Make an abrupt inflow disturbance, step to half hthe flow:
inflow(round(length(t_vector)/3):2*round(length(t_vector)/3),2) = -I*0.25;

% ocilationg inflow disturbance:
%inflow(:,2) = ( I/5*sin(0.01*t_vector)) ;

% Milling-line stop 100% till 50% på 5 min:
x = [1 3000 3300 6000 6300 10000];
v = [0 0 -I*0.5 -I*0.5 0 0];
%inflow(:,2) = interpn(x,v,1:10000);

figure
plot(inflow(:,2))


%% reference value:

ref = 31;
ref = [t_vector, ref*ones(length(t_vector),1)];

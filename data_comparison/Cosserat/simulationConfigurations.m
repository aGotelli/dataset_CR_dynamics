function [Const, Config] = simulationConfigurations()


%%  Kinematics rod base
Const.r_X0 = [
    0
    0
    0
];

%   Quaternion of rod base [w x y z]
Const.Q_X0 = [1, 0, 0, 0]';

Const.eta_X0  = zeros(6, 1);
Const.deta_X0 = zeros(6, 1);

%%  Wrench at tip
Const.Lambda_X1 = zeros(6, 1);


%%  Tendon Actuation

% Define a dynamic cable tension function as a function of time
Const.cable_tension = @(t) 0;%5*sin(2*pi*0.5*t) + 2.5*sin(2*pi*2*t);

% Positioning of tendon
d = 0.01;
Const.D = [
    0
    0
    d
];
Const.D_prime = zeros(3,1);



%%   Time integration settings
t_end = 3;
dt    = 10e-3;
Beta  = 1/4;
Gamma = 1/2;
a     = Gamma/(Beta*dt);
b     = 1/(Beta*dt^2);




%%  GEOMETRICAL PARAMETERS

%   Lenght of the rod
Const.L = 1.0;

%   Radius Cross section
Const.Rc  = 0.001;

%   Area
Const.Area = pi*Const.Rc^2;

%   Geometrical moment of inertia
Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;



%%  MATERIAL PARAMETERS

%   Specific weight
Const.rho = 7800;

%   Shear modulus
Const.G = 80e9;

%   Young modulus
Const.E  = 90e9;

%   Internal damping
Const.mu = 0*1e-3;

%   Value of gravity
Const.g = 9.81;





%%  STIFFNESS AND INERTIA OF CROSS SECTION

Const.GIxx = Const.G*Const.J(1,1);
Const.EIyy = Const.E*Const.J(2,2);
Const.EIzz = Const.E*Const.J(3,3);

Const.EA = Const.E*Const.Area;
Const.GA = Const.G*Const.Area;

Const.M_cal  = Const.rho*diag([Const.J(1, 1), Const.J(2, 2), Const.J(3, 3), Const.Area, Const.Area, Const.Area]);


Const.rhoAg = Const.rho*Const.Area*Const.g;

Const.fg = [
        0
        0
  -Const.rhoAg
];


Const.H_cal = diag([Const.GIxx, Const.EIyy, Const.EIzz, Const.EA, Const.GA, Const.GA]);

%%  STRAIN BASED PARAMETERIZATION


% Definition of actuated values
% K1, K2, K3, ...
% if value is 1 -> the variable is actuated
% if value is 0 -> the variable is not actuated
Const.V_a = [0, 1, 1, 0, 0, 0];

Const.dim_V_a = sum(Const.V_a);

%   Define the size of the parameterisation
ne = 3;
Const.dim_base_k = ne*Const.V_a;
%   Compute size of q
Const.dim_base   = Const.V_a*Const.dim_base_k';


%   Automatically define matrix B
M_selec = eye(6,6);
[~,col] = find(Const.V_a==1);
Const.B = M_selec(:,col);

%   Define constant strain
Const.Xi_c = [0;0;0;1;0;0];


%%   Store in Config
Config.t_end = t_end;
Config.dt    = dt;
Config.a     = a;
Config.b     = b;
Config.Beta  = Beta;
Config.Gamma = Gamma;

%   Properties for Cosserat ODEs integration
% Number of observation points
N_nodes = 30;
% Actual observationn points
[~, X_grid]=cheb(N_nodes-1, Const.L);

%   Saves as configuration
Config.forward_integration_domain = X_grid;
Config.backward_integration_domain = flip(X_grid);

%   Threshold residual norm
Config.r_min = 1e-5;

%   Value of the numerical perturbation
Config.delta = 1e-6;



%%  STIFFNESS AND DAMPING MATRICES

[Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config);

Const.Kee = Kee;
Const.Dee = Dee;






end
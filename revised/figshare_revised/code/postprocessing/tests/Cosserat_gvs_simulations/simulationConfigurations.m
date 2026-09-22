function [Const, Config] = simulationConfigurations(DoFs, t_end, dt)
% SIMULATIONCONFIGURATIONS Configure Cosserat rod simulation parameters
%
% INPUTS:
%   DoFs - (optional) Array of 6 elements specifying number of modes for each DOF
%          DoFs = [K1_modes, K2_modes, K3_modes, Gamma1_modes, Gamma2_modes, Gamma3_modes]
%          Where:
%            K1_modes     - Number of modes for torsion (around x-axis)
%            K2_modes     - Number of modes for bending (around y-axis) 
%            K3_modes     - Number of modes for bending (around z-axis)
%            Gamma1_modes - Number of modes for extension (along x-axis)
%            Gamma2_modes - Number of modes for shear (along y-axis)
%            Gamma3_modes - Number of modes for shear (along z-axis)
%          Example: [1, 3, 3, 0, 0, 0] = 1 torsion mode + 3 bending modes each for y,z
%
% OUTPUTS:
%   Const  - Structure containing physical constants and parameters
%   Config - Structure containing simulation configuration

% Set default DoFs if not provided
if nargin < 1
    DoFs = [0, 3, 0, 0, 0, 0];  % Default: [K1_modes, K2_modes, K3_modes, Gamma1_modes, Gamma2_modes, Gamma3_modes]
end

if nargin < 2
    t_end = 3;
    dt    = 5e-3;
end

%%   Time integration settings
Beta  = 1/4;
Gamma = 1/2;
a     = Gamma/(Beta*dt);
b     = 1/(Beta*dt^2);




%%  GEOMETRICAL PARAMETERS

%   Lenght of the rod
Const.L = 0.48;

%   Radius Cross section
Const.Rc  = 0.002;

%   Area
Const.Area = pi*Const.Rc^2;

%   Geometrical moment of inertia
Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;


















%   -> Specific weight


Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;

%   -> Damping coefficient
Const.mu = 1.8e-1;

%   -> Material properties
Const.GI = 0;
Const.EI = 0.088;

% Axial (extensional) and shear stiffnesses, E*A and G*A. Given this
% robot's actuation pattern (Config.V_a = [0,1,1,0,0,0]: only the two
% bending curvatures are actuated), Const.B in the block below selects
% only columns 2 and 3 of H_cal, so Const.EA/Const.GA are never actually
% read by computeGeneralisedStiffnessDampingMatrices.m -- only Const.EI
% (repeated for both bending directions) contributes to Kee/Dee. They
% are set here purely so that Const.H_cal below is well-defined; if this
% actuation pattern is ever changed to actuate extension or shear,
% replace these placeholders with measured/identified values first.
Const.EA = 1e6;
Const.GA = 1e6;

%   Gravity
Const.Gamma_g = 9.81;

%%  Tendon actuation

%   Positioning cable
d = 37.5e-3;
Const.D1 = [
    0
    0
    d
];
Const.D2 = [
    0
    d
    0
];



































%%  MATERIAL PARAMETERS

%   Specific weight
weight = 102; % [g]
weight = weight/1000;
Const.rho = weight/(pi*Const.Rc^2*Const.L);

%   Shear modulus
% Const.G = 80e9;
Const.G = 5.0293e5;

%   Young modulus
% Const.E  = 210e9;
Const.E  = 5.2203e+08;


%   Value of gravity
Const.g = 9.81;





%%  STIFFNESS AND INERTIA OF CROSS SECTION

Const.EIxx = 0.088;
Const.EIyy = 0.088;

%   Standard values in case the deformations DoFs are changed
Const.EIzz = Const.EIyy;
Const.GIxx = 80;
Const.EA = 1e6;
Const.GA = 1e6;

Const.M_cal  = Const.rho*diag([Const.J(1, 1), Const.J(2, 2), Const.J(3, 3), Const.Area, Const.Area, Const.Area]);


Const.rhoAg = Const.rho*Const.Area*Const.g;

Const.fg = [
        0
        0
  -Const.rhoAg
];


Const.H_cal = diag([Const.GIxx, Const.EIyy, Const.EIzz, Const.EA, Const.GA, Const.GA]);
% 
% Const.GI = 79;
% Const.EI = 3.5e7;
% Const.EA = 1.64e7;
% Const.GA = 6.34e6;
% Const.H_cal = diag([Const.GI, Const.EI, Const.EI, Const.EA, Const.GA, Const.GA]);

%%  STRAIN BASED PARAMETERIZATION (using DoFs parameter like Python)

% Store DoFs configuration
Const.DoFs = DoFs;  % [K1_modes, K2_modes, K3_modes, Gamma1_modes, Gamma2_modes, Gamma3_modes]

% Define deformations based on DoFs (1 if modes > 0, 0 otherwise)
% This matches Python: admitted_deformations = np.array([1 if dof > 0 else 0 for dof in DoFs])
Const.V_a = double(DoFs > 0);  % Actuated DOFs (1 if modes > 0, 0 otherwise)

Const.dim_V_a = sum(Const.V_a);

% Define the size of the parameterization using actual DoFs
% This matches Python: ne_i = np.array(DoFs) and dim_base_k = self.ne_i
Const.dim_base_k = DoFs;  % Number of modes per DOF

% Compute total number of generalized coordinates
% This matches Python: ne = int(np.sum(DoFs))
Const.ne = sum(DoFs);
Const.dim_base = Const.ne;  % Total number of generalized coordinates


% Automatically define matrix B (selection matrix)
% This matches Python: B = np.eye(6); actuated_cols = np.where(self.admitted_deformations == 1)[0]; self.B = B[:, actuated_cols]
M_selec = eye(6,6);
actuated_cols = find(Const.V_a == 1);
Const.B = M_selec(:, actuated_cols);

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
N_nodes = 31;
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
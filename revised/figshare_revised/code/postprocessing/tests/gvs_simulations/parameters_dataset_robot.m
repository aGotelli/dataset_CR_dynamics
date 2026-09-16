%   GEOMETRIC PARAMETERS
Config.L = 0.48;
Const.Rc = 0.002;

weight = 102; % [g]
weight = weight/1000;
specific_weight = weight/(pi*Const.Rc^2*Config.L);

%   -> Specific weight
Const.rho = specific_weight;

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

%%  Define constant matrices of the problem

V_a = Config.V_a;

M_selec = eye(6,6);

[~,col] = find(V_a==1);

Const.B     = M_selec(:,col);
Const.B_bar = M_selec;
Const.B_bar(:,col) = [];

[~,col] = size(col);

Const.dim_B = col;

%   Fixed (non-actuated) strain, and reference strain on the actuated
%   components (unit extension, zero torsion/shear).
Const.Xi_c = Const.B_bar'*[0;0;0;1;0;0];
Const.Xi_0 = Const.B'*[0;0;0;1;0;0];

Const.M      = zeros(3,3);
Const.M(1,1) = Const.rho*pi*Const.Rc^2;
Const.M(2,2) = Const.rho*pi*Const.Rc^2;
Const.M(3,3) = Const.rho*pi*Const.Rc^2;

Const.M_cal = [Const.rho*[Const.J,zeros(3)];[zeros(3),Const.M]];

Const.M_cal_prime = zeros(6);

Const.M_cal_bar = Const.B'*Const.M_cal*Const.B;

Const.G = Const.B*inv(Const.M_cal_bar)*Const.B';

%  STIFFNESS AND INERTIA OF THE CROSS SECTION
Const.H_cal = diag([0*Const.EI, Const.EI, Const.EI, Const.EA, Const.GA, Const.GA]);

[Kee,Dee] = computeGeneralisedStiffnessDampingMatrices(Const,Config);

Const.Kee = Kee;
Const.Dee = Dee;

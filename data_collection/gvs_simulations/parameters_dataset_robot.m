%   GEMETRIC PARAMETERS
Config.L = 0.48;
Const.Rc  = 0.002;

weight = 102; % [g]
weight = weight/1000;
specific_weight = weight/(pi*Const.Rc^2*Config.L);

%   -> Specific weigth 
Const.rho = specific_weight;


Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;



%   -> Damping coefficient
Const.mu = 1.8e-1;


%   -> Material Properties
Const.GI = 0;
% Const.EI = 0.092;
Const.EI = 0.088;

%   Keep zero
Const.EA = 0;
Const.GA = 0;

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



%%  Define constant matrices in the problem

V_a = Config.V_a;

M_selec = eye(6,6);
M_selec_K = zeros(3,3);

[row,col] = find(V_a==1);

Const.B = M_selec(:,col);
Const.B_bar = M_selec;
Const.B_bar(:,col)=[];

[row,col] = size(col);

Const.dim_B = col;

%   Fixed curvature
Const.Xi_c = Const.B_bar'*[0;0;0;1;0;0];
Const.Xi_0 = Const.B'*[0;0;0;1;0;0];


Const.M      = zeros(3,3);
Const.M(1,1) = Const.rho*pi*Const.Rc^2;
Const.M(2,2) = Const.rho*pi*Const.Rc^2;
Const.M(3,3) = Const.rho*pi*Const.Rc^2;

Const.M_cal       = [Const.rho*[Const.J,zeros(3)];[zeros(3),Const.M]];

Const.M_cal_prime = zeros(6);

Const.M_cal_bar   = Const.B'*Const.M_cal*Const.B;

Const.G           = Const.B*inv(Const.M_cal_bar)*Const.B';


%  STIFFNESS AND INERTIA OF CROSS SECTION
Const.H_cal = diag([0*Const.EI, Const.EI, Const.EI, Const.EA, Const.GA, Const.GA]);

[Kee, Dee] = computeGeneralisedStiffnessDampingMatrices(Const, Config);

Const.Kee = Kee;
Const.Dee = Dee;
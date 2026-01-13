
Const.rho = 1000;
Const.Rc  = 0.1;

Const.J      = zeros(3,3);
Const.J(1,1) = pi*Const.Rc^4/2;
Const.J(2,2) = pi*Const.Rc^4/4;
Const.J(3,3) = pi*Const.Rc^4/4;

% 
% I1 = (pi*Const.Rc^4)/2;%(pi*(2*Const.Rc)^4)/32;
% I2 = (pi*Const.Rc^4)/4;%(pi*(2*Const.Rc)^4)/64;
% I3 = (pi*Const.Rc^4)/4;%(pi*(2*Const.Rc)^4)/64;
% 

Aire = pi*Const.Rc^2;

% Gj = 10e4*Const.rho;
% E  = 10e4*Const.rho;

Const.mu = 1e-3;

% H = diag([Gj*I1,E*I2,E*I3,E*Aire,Gj*Aire,Gj*Aire]);
% H = diag([3.5e7,3.5e7,3.5e7,4.2e8,1.61e8,1.61e8]);

Const.GI = 79;
Const.EI = 3.5e7;
Const.EA = 1.64e7;
Const.GA = 6.34e6;


% definition des matrices constantes dans ce pb
% --------------------- %
% Constantes control %

V_a = Config.V_a;

M_selec = eye(6,6);
M_selec_K = zeros(3,3);

[row,col] = find(V_a==1);

Const.B = M_selec(:,col);
Const.B_bar = M_selec;
Const.B_bar(:,col)=[];

[row,col] = size(col);

Const.dim_B = col;

% Courbure imposée
Const.Xi_c = Const.B_bar'*[0;0;0;1;0;0];
Const.Xi_0 = Const.B'*[0;0;0;1;0;0];

%---------------%
% constante géométrique %

Const.M      = zeros(3,3);
Const.M(1,1) = Const.rho*pi*Const.Rc^2;
Const.M(2,2) = Const.rho*pi*Const.Rc^2;
Const.M(3,3) = Const.rho*pi*Const.Rc^2;

Const.M_cal       = [Const.rho*[Const.J,zeros(3)];[zeros(3),Const.M]];

Const.M_cal_prime = zeros(6);

Const.M_cal_bar   = Const.B'*Const.M_cal*Const.B;

Const.G           = Const.B*inv(Const.M_cal_bar)*Const.B';
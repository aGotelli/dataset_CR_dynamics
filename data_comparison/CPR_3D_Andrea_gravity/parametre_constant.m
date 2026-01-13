
Config.rho = 7800;
Config.Rc  = 0.001;

Gj = 80;
E  = 207e9;

Aire = pi*Config.Rc^2;
Config.Aire = Aire;

Config.J      = zeros(3,3);
Config.J(1,1) = pi*Config.Rc^4/2;
Config.J(2,2) = pi*Config.Rc^4/4;
Config.J(3,3) = pi*Config.Rc^4/4;

Config.GI = Gj*Config.J(2,2);
Config.EI = E*Config.J(2,2);
Config.EA = E*Aire;
Config.GA = Gj*Aire;

Config.robot.limb.H_cal = diag([Config.GI,Config.EI,Config.EI,Config.EA,Config.GA,Config.GA]);
% --------------------- %
% Configantes control %

V_a = Config.robot.limb.V_a;

M_selec = eye(6,6);
M_selec_K = zeros(3,3);

[row,col] = find(V_a==1);

Config.B = M_selec(:,col);
Config.B_bar = M_selec;
Config.B_bar(:,col)=[];

[row,col] = size(col);

Config.dim_B = col;

% Courbure imposée
Config.Xi_c = Config.B_bar'*[0;0;0;1;0;0];
% Config.Xi_0 = Config.B'*[0;0;0;1;0;0];
Config.Xi_0{1}  = [0;0;0;1;0;0];
Config.Xi_0{2}  = [0;0;0;1;0;0];
Config.Xi_0{3}  = [0;0;0;1;0;0];
Config.Xi_0{4}  = [0;0;0;1;0;0];
Config.Xi_0{5}  = [0;0;0;1;0;0];
Config.Xi_0{6}  = [0;0;0;1;0;0];
Config.Xi_0{7}  = [0;0;0;1;0;0];
Config.Xi_0{8}  = [0;0;0;1;0;0];
%---------------%
% Configante géométrique %

Config.M      = zeros(3,3);
Config.M(1,1) = Config.rho*pi*Config.Rc^2;
Config.M(2,2) = Config.rho*pi*Config.Rc^2;
Config.M(3,3) = Config.rho*pi*Config.Rc^2;

Config.robot.limb.M_cal_l = [[Config.rho*Config.J,zeros(3)];[zeros(3),Config.M]];

Config.robot.plaform.M_platform = 0.1;
Config.robot.plaform.R_platform = 0.1;
Config.robot.plaform.H_platform = 1e-3;


M = 0*Config.robot.plaform.M_platform;
R = Config.robot.plaform.R_platform;
H = Config.robot.plaform.H_platform;

J_z = M*R*R/2;
J_x = (M*R*R / 4) + (M*H*H/ 12);
J_y = J_x;

Config.robot.plaform.M_cal_p =  diag([J_x;J_y;J_z;M;M;M]);
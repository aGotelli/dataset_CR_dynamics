clc
clear all
close all

%------------- entete ----------------%

% ajoute le dossier outils
addpath('./outils')
addpath('./methode spectral')


% Initialisation boucle de temps
Config.dt    = 0.01;%1e-2;
Config.t_fin = 30;
Config.time  = -Config.dt:Config.dt:Config.t_fin;
Config.Nt    = length(Config.time);

% Définition des noeuds pour l'integration Spectrale
Config.N_noeuds = 30; % nombre de noeuds
% Critère résidu Newton
Config.Res_min = 1e-6;
Config.Res_com = 1e-2;

Config.simu.dyn = 0;

h = Config.dt;
Config.Integrateur.newmark.Gamma = 1/2;
Config.Integrateur.newmark.Beta = 1/4; % interpolation 1/4; % valeur moyenne 1/6

Config.Integrateur.newmark.a = Config.Integrateur.newmark.Gamma/(Config.Integrateur.newmark.Beta*h);
Config.Integrateur.newmark.b = 1/(Config.Integrateur.newmark.Beta*h^2);


% alpha Methode
Config.Integrateur.alpha_methode.rho_inf = 0.7;

%------------- fin entete ----------------%

%--- Configuration robot ----% 

% nombre de jambe
Config.robot.nl = 1;
% longueur de chaque jambe
Config.robot.limb.l = 1*ones(Config.robot.nl,1);

% rayon de la base et plateforme

Config.robot.r_base = 0.0;
% Config.robot.r_plaform = 0.1;

% Définition des variables actionnées
% K1, K2, K3, ...
% si valeur est à 1 -> la variable est actionnée
% si valeur est à 0 -> la variable n'est pas actionnée
Config.robot.limb.V_a = [0,1,1,0,0,0];
% Définition de la taille de la base pour chaque variable
ne_i = 3;
Config.robot.limb.dim_base_k = ne_i*[0,1,1,0,0,0];
% Calcul de la taille de qe_i
Config.robot.limb.dim_qe_i   = Config.robot.limb.V_a*Config.robot.limb.dim_base_k';


% Valeur de la gravité
Config.Gamma_g = 9.81;

% Parametre constants de la simulation
% Valeurs physique de la poutre 
parametre_constant;
if Config.robot.nl==0
    rz_p = 0.5;
    qr = [];
    Var.lambda{1}  = 0;
else
    % Pose initial de la plateforme
    rz_p = sqrt(Config.robot.limb.l(1)^2 - (Config.robot.r_base - Config.robot.r_plaform)^2);
    qr =  -asin((Config.robot.r_base - Config.robot.r_plaform)./Config.robot.limb.l);
end

% pendant dans la gravité
% rz_p = -sqrt(Config.robot.limb.l(1)^2 - (Config.robot.r_base - Config.robot.r_plaform)^2);
% qr =  pi+asin((Config.robot.r_base - Config.robot.r_plaform)./Config.robot.limb.l);

% Config Liaison Ai et Aip
for it_nl = 1 :Config.robot.nl
    % Liaison Base/Limb
    [A,bar_A] = type_liaison('R'); % 'R' Rotoid, 'S' spherical, 'C' clamped
    Config.robot.limb.Aoi{it_nl} = A;
    Config.robot.limb.bar_Aoi{it_nl} = bar_A;
    % Liaison Platform/Limb
    [A,bar_A] = type_liaison('S'); % 'R' Rotoid, 'S' spherical, 'C' clamped
    Config.robot.limb.Api{it_nl} = A;
    Config.robot.limb.bar_Api{it_nl} = bar_A;
end

% ---------------------- Init -------------------------ù
% Initialisation de la base q
Var.q         = [zeros(Config.robot.nl,1);zeros(Config.robot.nl*Config.robot.limb.dim_qe_i,1)];
Var.q         = [qr;zeros(Config.robot.nl*Config.robot.limb.dim_qe_i,1)];
Var.size.q    = (Config.robot.nl)*(Config.robot.limb.dim_qe_i+1);

% Initialisation de mu_p, pose de la base
% Var.mu_p = [Log_SO3_(Config.robot.plaform.Rp);Config.robot.plaform.rp];  
Var.mu_p = [0.;0;0;0;0;0];
Var.size.mu_p =6;

Var.Rp = Exp_SO3_([0;0;0]);
Var.rp = [0;0;rz_p];
Var.g_0 = [Var.Rp,Var.rp;0 0 0 1];

% Initialisation de lambda
Var.size.lambda = 0;

for it_nl = 1 :Config.robot.nl
    dim_A_bar = size(Config.robot.limb.bar_Api{it_nl});
    Var.lambda{it_nl}  = zeros(dim_A_bar(1),1);
    Var.size.lambda = Var.size.lambda + dim_A_bar(1);
end

% Config position g0 limb et 2R3(i) et gpi

% theta = 2*pi/Config.robot.nl;
% 
% for it_nl = 1 :Config.robot.nl
%     Config.robot.limb.roi{it_nl} = Config.robot.r_base*[cos((it_nl-1)*theta);sin((it_nl-1)*theta);0];
%     Config.robot.limb.Roi{it_nl} = [cos((it_nl-1)*theta) -sin((it_nl-1)*theta)  0; sin((it_nl-1)*theta) cos((it_nl-1)*theta)  0;  0 0 1]*[0 1 0;-1 0 0; 0 0 1]*[0 0 -1; 0 1 0; 1 0 0];
%     Config.robot.limb.R3i{it_nl} = eye(3);
%     Config.robot.limb.r3i{it_nl} = [0;0;0];
%     Config.robot.plaform.rpi_p{it_nl} = Config.robot.r_plaform*[cos((it_nl-1)*theta);sin((it_nl-1)*theta);0];
%     Config.robot.plaform.Rpi_p{it_nl} = [cos((it_nl-1)*theta) -sin((it_nl-1)*theta)  0; sin((it_nl-1)*theta) cos((it_nl-1)*theta)  0;  0 0 1]*[0 1 0;-1 0 0; 0 0 1]*[0 0 -1; 0 1 0; 1 0 0];
% end

theta_oi = [0; 0;2*pi/3;2*pi/3;-2*pi/3;-2*pi/3];
theta_pi = [-pi/3;pi/3;pi/3;pi;pi;-pi/3];

% theta_oi = [0;2*pi/3;2*pi/3;-2*pi/3;-2*pi/3;0];
% theta_pi = [pi/3;pi/3;pi;pi;-pi/3;-pi/3];

Config.robot.limb.qoi_0 = 2*pi*ones(6,1) ;
Config.robot.limb.qoi_f1 = pi/2*ones(6,1) +  [5.86; 5.56; 5.86; 5.46; 5.86; 5.26];
% Config.robot.limb.qoi_f2 = pi/2*ones(6,1) +  [7.95; 7.65; 4.29; 3.89; 3.77; 3.17];
Config.robot.limb.qoi_f2 = Config.robot.limb.qoi_f1  + [2*pi/3;2*pi/3;-pi/2;-pi/2;-2*pi/3;-2*pi/3];
% Config.robot.limb.qoi_f1 = pi/2*ones(6,1) +  [5.86; 5.46; 5.86; 5.26; 5.86; 5.56];
% Config.robot.limb.qoi_f2 = pi/2*ones(6,1) +  [7.95; 3.89; 4.29; 3.17; 3.77; 7.65];
% Config.robot.limb.qoi_f2 = Config.robot.limb.qoi_f1 + ( [7.95; 7.65; 4.29; 3.89; 3.77; 3.17] - [5.86; 5.56; 5.86; 5.46; 5.86; 5.26] );

pas = 2*pi/(Config.robot.nl);
theta_oi = [0:pas:2*pi-pas];
theta_pi = [0:pas:2*pi-pas];

for it_nl = 1 :Config.robot.nl
    Config.robot.limb.roi{it_nl} = Config.robot.r_base*[cos(theta_oi(it_nl));sin(theta_oi(it_nl));0];
    Config.robot.limb.Roi{it_nl} = [cos(theta_oi(it_nl)) -sin(theta_oi(it_nl))  0; sin(theta_oi(it_nl)) cos(theta_oi(it_nl))  0;  0 0 1]*[0 1 0;-1 0 0; 0 0 1]*[0 0 -1; 0 1 0; 1 0 0];
    Config.robot.limb.R3i{it_nl} = eye(3);
    Config.robot.limb.r3i{it_nl} = [0;0;0];
    Config.robot.plaform.rpi_p{it_nl} = Config.robot.r_plaform*[cos(theta_pi(it_nl));sin(theta_pi(it_nl));0];
    Config.robot.plaform.Rpi_p{it_nl} = [cos(theta_pi(it_nl)) -sin(theta_pi(it_nl))  0; sin(theta_pi(it_nl)) cos(theta_pi(it_nl))  0;  0 0 1]*[0 1 0;-1 0 0; 0 0 1]*[0 0 -1; 0 1 0; 1 0 0];
end

% ---------------------- Tracé -------------------------ù
Var.t = 0;
trace_CPR_3D(Config,Var)

% ---------------------- intégration temporelle -------------------------ù
name = "3D_CPR_N30_rho_07_t_30";
Config.plot_NR = false;
Config.plot_t  = false;
if(Config.plot_t)
    Var.video = VideoWriter(name + ".avi");
    open(Var.video)
end

tic
[T,Xi,lambda] = Time_Integration_Newton_Dynamic_CPR_3D_Spectral(Config,Var);
tsimul = toc
save(name + ".mat");
close(Var.video);

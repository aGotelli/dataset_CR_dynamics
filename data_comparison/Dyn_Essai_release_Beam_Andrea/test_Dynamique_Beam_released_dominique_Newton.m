clc
clear all
close all

% ajoute le dossier outils
addpath('./outils')
addpath('./IDM')
addpath('./methode spectral')

Config.option=odeset('RelTol',10^(-8),'AbsTol',10^(-8));

% Variable pour visualisation du transitoire
% 0 pas de visualisation
% 1 visualisation
Config.Trace_transitoire = 0;
    
% Initialisation des variables pour le tracé
r_dot_trace_save = [];
T_trace_save = [];
x_trace_save = [];
y_trace_save = [];
z_trace_save = [];
           
% Initialisation boucle de temps
Config.dt    = 0.005;
Config.t_fin = 10.0;
Config.time  = 0:Config.dt:Config.t_fin;
Config.Nt    = length(Config.time);


% longueur de la poutre
Config.L = 10;
% Pas de discrétisation du corps
Config.dX = Config.L/101;
% Construction du corps et de ses paramètres
Config.X  = 0:Config.dX:Config.L;
% Nombre de noeuds discrétisant le corps
Config.Nx = length(Config.X);

% Définitio des variables actionnées
% K1, K2, K3, ...
% si valeur est à 1 -> la variable est actionnée
% si valeur est à 0 -> la variable n'est pas actionnée
Config.V_a = [0,1,0,0,0,0];
% Définition de la taille de la base pour chaque variable
Const.dim_base_k = 3*[0,1,0,0,0,0];
% Calcul de la taille de q 
Const.dim_base   = Config.V_a*Const.dim_base_k';

% Valeur de la gravité
Const.Gamma_g = 9.81;


% Parametre constants de la simulation
% Valeurs physique de la poutre 
parametre_constant_Compare_Dominique_F_ABS;

% Initialisation du temps à 0
time = 0;

% Position et orientation initiales de la tete
r_0 = [0;0;0];
q_0 = [1 0 0 0]';
q_0 = [0.7071068 0 0.7071068 0]';
eta_0 = [0 0 0 0 0 0]'; % vitesse de la tête

% ------------ Calcul des gain elastique et ammortissement --------------- %

% ------ Initialisation ----- %
Kee_0 = zeros(Const.dim_base*Const.dim_base,1);
Dee_0 = zeros(Const.dim_base*Const.dim_base,1);

% ------- Calcul ------ %
CI = [Kee_0; Dee_0];
[X Y1] = ode45(@(t,y) elastic_gain(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

% ------- Affectation des résultats -----------%
Kee_vec = Y1(end,1:Const.dim_base*Const.dim_base);
Dee_vec = Y1(end,1+Const.dim_base*Const.dim_base:2*Const.dim_base*Const.dim_base);

Kee = reshape(Kee_vec, [Const.dim_base Const.dim_base]);
Dee = reshape(Dee_vec, [Const.dim_base Const.dim_base]);

Const.Kee = Kee;
% Const.Dee = Dee;
Const.Dee = 0*1e-3*Kee;

% ------------------------------------------------------- %

% Initialisation de la base q
q0 = zeros(Const.dim_base,1);

% Définiton de la variation de la force en bout 
C_1 = [0:0.1:0.5];

% Taille du vecteur de force
% size_Force = max(size(C_1));
% 
% % Boucle d'increment en force
% for it_Force = 1:size_Force
%     
%     % Initalisation du vecteur F_1
%     Const.F1 = zeros(6,1);
% %     Const.F1(1,1) = C_1(it_Force);
% %     Const.F1(2,1) = C_1(it_Force)/3;
% %     Const.F1(3,1) = C_1(it_Force)/6;
%     Const.F1(4,1) = C_1(it_Force);
%     
%     % Affichage de F_1
%     Const.F1
%     
%     % Valeur initiale de l'intégrale 
%     CI = q0;
%     
%     % Calcul de l'intégrale de q_dot
%     [T Y2_s] = ode45(@(t,y) Calcul_q_dot(t,y,time,Const,Config),[0 0.005],CI,Config.option); % Solve ODE
%    
%     % affectation des sorties
%     q0 = Y2_s(end,:)';
%     
%     % Reconstruction de la poutre pour visualisation
%     
%     Const.q = Y2_s(end,:)';
%     Const.q_dot = 0*Y2_s(end,:)';
%     CI = [q_0;r_0;eta_0];
%      
%     [X Y3_s] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE
%      
%     % Affectation des valeurs pour le tracé
%     Xt = Y3_s(:,5:7);
%     
%     x_trace_save = [x_trace_save,Xt(:,1)];
%     y_trace_save = [y_trace_save,Xt(:,2)];
%     z_trace_save = [z_trace_save,Xt(:,3)];
%     X_trace_save = X;
%     
%  % Tracé de la poutre dans l'espace 3D
% %     if mod(it_Force,10)==0
%         figure(5)
%         plot3(Xt(:,1),Xt(:,2),Xt(:,3),'b','LineWidth',2)
%         axis equal
%         xlim([-Config.L Config.L])
%         ylim([-Config.L Config.L])
%         zlim([-Config.L Config.L])
%         grid on
%         hold on
%         drawnow
% %     end
% 
% end
%%
%---------------------------------- %
%
% Integration temporelle

% Initialisation de la base q
% Const.q         = Y2_s(end,:)';
Const.q         =[  -0.2; 0.2; -0.2];

% Const.q(1,1) = 1; % J -> q=1; C -> q=3; O -> q=7.
% Const.r(Const.dim_base_k(1)+1,1) = 1;

Const.q_dot     = zeros(Const.dim_base,1);

Const.q_dot_dot = zeros(Const.dim_base,1);

Const.eta_dot   = zeros(6,1);
Const.eta_dot_0   = zeros(6,1);

Const.r_0 = r_0;
Const.q_0 = q_0; 
Const.eta_0 = eta_0;
Const.F1 = zeros(6,1);
CI = [Const.q;Const.q_dot];

tic
[T,q,q_dot,q_dot_dot] = Time_integration_Newton_beam_released_spectral(Config.time,Const,Config);
tsimul = toc
save('Beam_released_dominique_3_modes_flexion_spectral_N_30_Newton_dt005.mat');

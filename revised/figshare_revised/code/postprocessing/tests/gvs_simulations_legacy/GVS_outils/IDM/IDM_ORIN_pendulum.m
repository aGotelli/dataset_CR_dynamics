function [F0,Q_a] = IDM_ORIN_pendulum(q_0,r_0,eta_0,time,Const,Config)

% calcul l'accélération de la tête eta_0_dot
% réalise les 3 intégration dans l'espace
% 2 Forward, 1 Backward

% First Foward ODE in Space 0 -> 1

%------------ Conditions initiales ----------%
CI = [q_0;r_0;eta_0];


[T Y1] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE


% Backward ODE in Space 1 -> 0

%------------ Conditions initiales ----------%

q_1   = Y1(end,1:4)';
r_1   = Y1(end,5:7)';
eta_1 = Y1(end,8:13)';

F_tilde_1 = ad_(eta_1)'*(Const.M_cal*eta_1);
S_1 = -diag([Const.m1,Const.m1,Const.m1,0,0,0]);

S_1 = reshape(S_1,[36,1]);

[~,F_1,~,~] = Forces_exterieur_pendulum(1,q_1,r_1,eta_1,Const,Config); %F_ext;
s_1 = F_1+[0;0;0;Const.m1*hat_(eta_1(1:3))*eta_1(4:6)];

CI = [q_1;r_1;eta_1;F_tilde_1;S_1;s_1];

[X Y2] = ode45(@(t,y) Backward_Sweep_IDM_pendulum(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

% Second Foward ODE in Space 0 -> 1

%------------ Conditions initiales ----------%
%     q_0       = X2(end,1:4)'  ; % orientation de la section
%     r_0       = X2(end,5:7)'  ; % position de la section
%     eta_0     = X2(end,8:13)' ; % vitesse de la section
%     F_tilde_0 = X2(end,14:19)' ; % Effort de la section

F_tilde_0 = ad_(eta_0)'*(Const.M_cal*eta_0);%X2(end,14:19)'; 
S_0_vec = Y2(end,20:55)'; % Matrice S en Vecteur
S_0     = reshape(S_0_vec,[6 6]);% Matrice S
s_0     = Y2(end,56:61)'; % vecteur s

eta_0_dot = Const.eta_dot;

Q_a = zeros(Const.dim_base,1);

CI = [q_0;r_0;eta_0;F_tilde_0;S_0_vec;s_0;eta_0_dot;Q_a];

[X Y3] = ode45(@(t,y)Forward_eta_dot_IDM_ORIN_pendulum(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

% Sorties

F0 = -S_0*eta_0_dot - s_0;
Q_a = Y3(end,68:67+Const.dim_base);

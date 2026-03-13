function eta_0_dot = f_eta_dot_IDM(q_0,r_0,eta_0,time,Const,Config)

% calcul l'accélération de la tête eta_0_dot
% réalise les 3 intégration dans l'espace
% 2 Forward, 1 Backward

% First Foward ODE in Space 0 -> 1

%------------ Conditions initiales ----------%
CI = [q_0;r_0;eta_0];

[T X1] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),[0 1],CI,Config.option); % Solve ODE

% Backward ODE in Space 1 -> 0

%------------ Conditions initiales ----------%

q_1   = X1(end,1:4)';
r_1   = X1(end,5:7)';
eta_1 = X1(end,8:13)';

F_tilde_1 = ad_(eta_1)'*(Const.M_cal*eta_1);
S_1 = zeros(36,1);

[~,F_1,~,~] = Forces_exterieur(1,q_1,r_1,eta_1); %F_ext;
s_1 = F_1;

CI = [q_1;r_1;eta_1;F_tilde_1;S_1;s_1];

[T X2] = ode45(@(t,y) Backward_Sweep_IDM(t,y,time,Const,Config),[0 1],CI,Config.option); % Solve ODE

% Second Foward ODE in Space 0 -> 1

%------------ Conditions initiales ----------%
% q_0       = X2(end,1:4)'  ; % orientation de la section
% r_0       = X2(end,5:7)'  ; % position de la section
% eta_0     = X2(end,8:13)' ; % vitesse de la section
% F_tilde_0 = X2(end,14:19)' ; % Effort de la section

F_tilde_0 = ad_(eta_0)'*(Const.M_cal*eta_0);
S_0_vec = X2(end,20:55)'; % Matrice S en Vecteur
S_0     = reshape(S_0_vec,[6 6]);% Matrice S
s_0     = X2(end,56:61)'; % vecteur s

[F_0,~,~,~] = Forces_exterieur(0,q_0,r_0,eta_0); %F_ext;

eta_0_dot = inv(-S_0)*(s_0 + F_0);

% 
% CI = [q_0;r_0;eta_0;F_tilde_0;S_0_vec;s_0;eta_0_dot];
% 
% [T X3] = ode45(@(t,y) Forward_eta_dot_IDM(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE
% 
% eta_0_dot = X3(1,62:67)'; % acceleration de la section

time
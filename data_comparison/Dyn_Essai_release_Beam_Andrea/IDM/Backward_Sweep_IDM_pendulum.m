function  dy = Backward_Sweep_IDM_pendulum(X,y,t,Const,Config)

% calcul l'ode bacward spaciale 
% Integration spaciale de 1->0
% entrées
% q       = y(1:4)  ; % orientation de la section
% r       = y(5:7)  ; % position de la section
% eta     = y(8:13) ; % vitesse de la section
% F_tilde = y(14:19); % Effort de la section
% S       = y(20:55); % Matrice S
% s       = y(56:61); % vecteur s
% sorties
% dy(1:4,1)   = q_prime  ; % orientation de la section
% dy(5:7,1)   = r_prime  ; % position de la section
% dy(8:13,1)  = eta_prime ; % vitesse de la section
% dy(14:19,1) = F_tilde_prime ; % Effort de la section
% dy(20:55,1) = S_prime; % Matrice S
% dy(56:61,1) = s_prime; % vecteur s

%-------------- entrées -----------------%

Q       = y(1:4)  ; % orientation de la section
r       = y(5:7)  ; % position de la section
eta     = y(8:13) ; % vitesse de la section
F_tilde = y(14:19); % Effort de la section
S_vec   = y(20:55); % Matrice S
s       = y(56:61); % vecteur s

S = reshape(S_vec,[6 6]);

%-------------- Intégration à rebourt  -----------------%
%-------------- changement de variable -----------------%
        %-------------- X -> 1-X -----------------%
%-------------- x' = f(x) -> x' = -f(x) -----------------%

X = Config.L-X; %

%-------------- forçage -----------------%

Phi = Base_Phi(X,t,Const,Config);

Xi_a = Phi'*Const.q;
Xi_dot_a = Phi'*Const.q_dot;
Xi_dot_dot_a = Phi'*Const.q_dot_dot;

Xi = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
Xi_dot = Const.B*Xi_dot_a;
Xi_dot_dot = Const.B*Xi_dot_dot_a;
%-------------- calculs -----------------%


M_cal = Const.M_cal;
M_cal_prime = Const.M_cal_prime;

[~,~,F_bar,F_bar_prime] = Forces_exterieur_pendulum(X,Q,r,eta,Const,Config);

Pxx = -ad_(Xi);
Pxy = zeros(6,6);
Pyx = M_cal;
Pyy = ad_(Xi)';

px = Xi_dot_dot - ad_(Xi_dot)*eta;
py = - F_tilde - F_bar;

q_prime   = quaternion_dot(Q,Xi(1:3));
r_prime   = r_dot(Q,Xi(4:6));
eta_prime = -ad_(Xi)*eta + Xi_dot;

F_tilde_prime = F_bar_prime + ad_(eta_prime)'*M_cal*eta + ad_(eta)'*(M_cal_prime*eta + M_cal*eta_prime);

S_prime = Pyx - S*Pxx + Pyy*S - S*Pxy*S;
s_prime = Pyy*s + py -S*Pxy*s - S*px;


        %-------------- sorties -----------------%
%-------------- Intégration à rebourt  -----------------%
%-------------- changement de variable -----------------%
        %-------------- X -> 1-X -----------------%
%-------------- x' = f(x) -> x' = -f(x) -----------------%

S_prime_vec = reshape(S_prime,[36,1]);

dy(1:4,1)   = -q_prime  ; % orientation de la section
dy(5:7,1)   = -r_prime  ; % position de la section
dy(8:13,1)  = -eta_prime ; % vitesse de la section
dy(14:19,1) = -F_tilde_prime ; % Effort de la section
dy(20:55,1) = -S_prime_vec; % Matrice S
dy(56:61,1) = -s_prime; % vecteur s



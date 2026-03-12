function   dy = Forward_eta(X,y,t,Const,Config)

% calcul la première ode spaciale 
% Integration spaciale de 0->1
% entrées
% q   = y(1:4); % orientation de la section
% r   = y(5:7); % position de la section
% eta = y(8:13); % vitesse de la section
% sorties
% dy(1:4,1)  = q_prime; % prientation de la section
% dy(5:7,1)  = r_prime; % position de la section
% dy(8:13,1) = eta_prime; % vitesse de la section


%-------------- entrées -----------------%
Q   = y(1:4); % orientation de la section
r   = y(5:7); % position de la section
eta = y(8:13); % vitesse de la section

%-------------- forçage -----------------%
Phi = Base_Phi(X,t,Const,Config);

Xi_a = Phi'*Const.q;
Xi_dot_a = Phi'*Const.q_dot;
% 
% for it_Xi=1:sum(Config.V_a)
%     Xi_a(it_Xi,1) = interp1(Config.X,Const.X_a_n(it_Xi,:),X,'spline');
%     Xi_dot_a(it_Xi,1) = interp1(Config.X,Const.X_dot_dot_a_n(it_Xi,:),X,'spline');
% %     Xi_dot_dot_a_n(it_Xi,1) = interp1(Config.X,Const.X_dot_dot_a_n(it_Xi,:),X,'spline');
% end

Xi = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + 0*Const.B*Const.Xi_0;
Xi_dot = Const.B*Xi_dot_a;

%-------------- calculs -----------------%
%

q_prime   = quaternion_dot(Q,Xi(1:3));
r_prime   = r_dot(Q,Xi(4:6));
eta_prime = -ad_(Xi)*eta + Xi_dot;

%-------------- sorties -----------------%

dy(1:4,1)  = q_prime; % orientation de la section
dy(5:7,1)  = r_prime; % position de la section
dy(8:13,1) = eta_prime; % vitesse de la section



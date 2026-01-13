function   dy = int_par_J(X,y,t,Const,Config)


%-------------- entrées -----------------%
%-------------- entrées -----------------%
Q = y(1:4); % orientation de la section
r = y(5:7); % position de la section
M = y(8:7+6*Const.dim_base);
M = reshape(M,[6 Const.dim_base]);
%-------------- forçage -----------------%
Phi = Base_Phi(X,t,Const,Config);

Xi_a = Phi'*Const.q;
Xi = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;

% figure(1)
% plot(X,Xi,'b.')
% hold on
% grid on
% drawnow
% 
% figure(2)
% plot(X,reshape(Const.A*Phi',[6*Const.dim_base, 1 ]),'b.')
% hold on
% grid on
% drawnow
%-------------- calculs -----------------%

%

Q_prime = quaternion_dot(Q,Xi(1:3));
r_prime = r_dot(Q,Xi(4:6));

R = quaternion_to_matrice(Q);
Ad_g=Ad_g_(R,r);

M_prime = Ad_g*Const.B*Phi';
%-------------- sorties -----------------%
dy(1:4,1)  = Q_prime; % orientation de la section
dy(5:7,1)  = r_prime; % position de la section
dy(8:7+6*Const.dim_base,1) = reshape(M_prime,[6*Const.dim_base, 1 ]); % vitesse de la section

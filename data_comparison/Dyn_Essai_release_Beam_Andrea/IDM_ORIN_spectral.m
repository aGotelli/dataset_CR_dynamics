function [F0,Q_a] = IDM_ORIN_spectral(q_0,r_0,eta_0,time,Const,Config)

% Définition des noeuds
N_noeuds = 30; % nombre de noeuds

% % Calcul des Noeuds sur la grille [0,L] legendre
% [x,D_Q]=legDc(N_noeuds-1);  % sur une grille [-1,1]
% X_grille=Config.L*(-x+1)/2; % sur une grille [0;L]
% DX =(-2/Config.L)*D_Q;      % Matrice de différentiation sur une grille [0;L]
eta_dot_0 = zeros(6,1);

[DX,X_grille]=cheb(N_noeuds-1,Config.L);  % sur une grille [0,L]

%calcul des Xi, Xi_dot et Xi_dot_dot sur les noeuds
for it_x = 1:N_noeuds
    
    Phi(:,:,it_x) = Base_Phi(X_grille(it_x),time,Const,Config);

    Xi_a = Phi(:,:,it_x)'*Const.q;
    Xi_dot_a = Phi(:,:,it_x)'*Const.q_dot;
    Xi_dot_dot_a = Phi(:,:,it_x)'*Const.q_dot_dot;

    Xi(:,it_x) = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
    Xi_dot(:,it_x) = Const.B*Xi_dot_a;
    Xi_dot_dot(:,it_x) = Const.B*Xi_dot_dot_a;
end

% Calcul de Q

CI = q_0;
CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];

f_A = @(it) quaternion_dot2(Xi(:,it));
f_B = @(it) zeros(max(size(CI)),1);

QX = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);


% calcul de r

CI = r_0;
CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];

f_A = @(it) zeros(max(size(CI)),max(size(CI)));
f_B = @(it) r_dot(QX(:,it),Xi(4:6,it));

rX = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

% calcul de eta

CI = eta_0;
CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

f_A = @(it) -ad_(Xi(:,it));
f_B = @(it) Xi_dot(:,it);

eta_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

% calcul de eta_dot

CI = eta_dot_0;
CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

f_A = @(it) -ad_(Xi(:,it));
f_B = @(it) Xi_dot_dot(:,it)- ad_(Xi_dot(:,it))*eta_X(:,it);

eta_dot_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

% calcul de Lambda
M_cal = Const.M_cal;
   
for it_x = 1:N_noeuds
    [~,~,F_bar(:,it_x),~] = Forces_exterieur(X_grille(it_x),QX(:,it_x),rX(:,it_x),eta_X(:,it_x),Const,Config);
end

[~,CI,~,~] = Forces_exterieur(X_grille(end),QX(:,end),rX(:,end),eta_X(:,end),Const,Config);
CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];

f_A = @(it) ad_(Xi(:,it))';
f_B = @(it) M_cal*eta_dot_X(:,it) - ad_(eta_X(:,it))'*M_cal*eta_X(:,it) - F_bar(:,it);

Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

F0 = -Lambda_X(:,1);

% calcul de Q_a

CI =  zeros(Const.dim_base,1);
CL_ind=[];
for i=1:Const.dim_base
    CL_ind=[CL_ind,i*N_noeuds];
end

f_A = @(it) zeros(Const.dim_base,Const.dim_base);
f_B = @(it)  -Phi(:,:,it)*Const.B'*Lambda_X(:,it);

Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

Q_a = -Qa_X(:,1);

function [F0,Q_a,J] = TIDM_spectral(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config)

J00 = [];
Je0 = [];
J0e = [];
Jee = [];

% Définition des noeuds
N_noeuds = 30; % nombre de noeuds

% % Calcul des Noeuds sur la grille [0,L] legendre
% [x,D_Q]=legDc(N_noeuds-1);  % sur une grille [-1,1]
% X_grille=Config.L*(-x+1)/2; % sur une grille [0;L]
% DX =(-2/Config.L)*D_Q;      % Matrice de différentiation sur une grille [0;L]

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

% Calcul de Quaternion

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
    [~,F1,F_bar(:,it_x),~] = Forces_exterieur(X_grille(it_x),QX(:,it_x),rX(:,it_x),eta_X(:,it_x),Const,Config);
end

CI = F1;
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
f_B = @(it) -Phi(:,:,it)*Const.B'*Lambda_X(:,it);

Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

Q_a = -Qa_X(:,1);

% Partie Variations

% on construit les matrice J00, Je0 de la jacobienne J
% Pour celà on fait varier \delta_psy_0 = \delta_i et on a \delta_q = 0
% Comme Delta_q = 0 alors Delta_xi = Delta_xi_dot = Delta_xi_dot_dot = 0

[R_0] = quaternion_to_matrice(q_0);
g_0 =[R_0,r_0;0 0 0 1];

[Rn_0] = quaternion_to_matrice(qn_0);
gn_0 =[Rn_0,rn_0;0 0 0 1];

gn = gn_0\g_0;

Rn = gn(1:3,1:3);
rn = gn(1:3,4);

var_theta = Log_SE3_(Rn,rn);
% Theta = Log_SO3_(Rn);

% variation sur Delta_psi
for it_D_psi = 1:6

    % calcul de Delta_psi

    Delta_psi_0 = zeros(6,1);
    Delta_psi_0(it_D_psi,1) = 1;
    
    CI = Delta_psi_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) zeros(6,1); % Delta_Xi(:,it);

    Delta_psi_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
     % calcul de Delta_eta
    Theta = var_theta(1:3,1);
    D = var_theta(4:6,1);
    
    Delta_eta_0 = a*T_SE3_m1_(Theta,D)*Delta_psi_0;
% %     Delta_eta_0 = [[a*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_0(4:6)),a*R_0']]*Delta_psi_0;
%     Delta_eta_0 = [[a*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_0(4:6)),a*eye(3)]]*Delta_psi_0;
    
    CI = Delta_eta_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) zeros(6,1); % -ad_(Delta_Xi(:,it))*eta_X(:,it)+ Delta_Xi_dot(:,it);

    Delta_eta_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    % calcul de Delta_eta_dot
    
    Theta = var_theta(1:3,1);
    D = var_theta(4:6,1);
    Delta_eta_dot_0 = b*T_SE3_m1_(Theta,D)*Delta_psi_0;
% %     Delta_eta_dot_0 = [[b*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_dot_0(4:6)+hat_(eta_0(1:3))*eta_0(4:6))+a*hat_(eta_0(4:6))*T_SO3_m1_(Theta)-hat_(eta_0(1:3))*hat_(eta_0(4:6)),b*R_0'-a*hat_(eta_0(1:3))*R_0']]*Delta_psi_0;
%     Delta_eta_dot_0 = [[b*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_dot_0(4:6)+hat_(eta_0(1:3))*eta_0(4:6))+a*hat_(eta_0(4:6))*T_SO3_m1_(Theta)-hat_(eta_0(1:3))*hat_(eta_0(4:6)),b*eye(3)-a*hat_(eta_0(1:3))]]*Delta_psi_0;
%     
    CI = Delta_eta_dot_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) -ad_(Xi_dot(:,it))*Delta_eta_X(:,it); % -ad_(Delta_Xi_dot(:,it))*eta_X(:,it)-ad_(Delta_Xi(:,it))*eta_dot_X(:,it)+ Delta_Xi_dot_dot(:,it);

    Delta_eta_dot_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    % calcul de Delta_Lambda
    M_cal = Const.M_cal;
    Aire = 0;
    Gamma_g = Const.Gamma_g;
    
    for it_x = 1:N_noeuds
        R = quaternion_to_matrice(QX(:,it_x));
        Delta_F_bar(:,it_x)= [zeros(3,1);hat_(Delta_psi_X(:,it_x))'*R'*[0;   0; -Const.rho*Aire*Gamma_g]];
    end
    Delta_F_bar(:,end)= [zeros(3,1);hat_(Delta_psi_X(:,it_x))'*R'*Const.F1(4:6)];

    CI = Delta_F_bar(:,end);
    CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];

    f_A = @(it) ad_(Xi(:,it))';
    f_B = @(it) M_cal*Delta_eta_dot_X(:,it) - ad_(Delta_eta_X(:,it))'*M_cal*eta_X(:,it) - ad_(eta_X(:,it))'*M_cal*Delta_eta_X(:,it) - Delta_F_bar(:,it);%ad_(Delta_Xi(:,it))'*Lambda_X(:,it)

    Delta_Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

    Delta_F0 = -Delta_Lambda_X(:,1);

    % calcul de Q_a

    CI =  zeros(Const.dim_base,1);
    CL_ind=[];
    for i=1:Const.dim_base
        CL_ind=[CL_ind,i*N_noeuds];
    end

    f_A = @(it) zeros(Const.dim_base,Const.dim_base);
    f_B = @(it)  -Phi(:,:,it)*Const.B'*Delta_Lambda_X(:,it);

    Delta_Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

    Delta_Qa = -Delta_Qa_X(:,1);
    J00 = [J00,Delta_F0];
    Je0 = [Je0,Delta_Qa];
end

%variation sur Delta_q 


for it_D_q = 1:Const.dim_base
    
    Delta_q = zeros(Const.dim_base,1);
    Delta_q(it_D_q,1) = 1;
    
    for it_x = 1:N_noeuds
        Delta_Xi(:,it_x) = Const.B*Phi(:,:,it_x)'*Delta_q;     
    end
    
    Delta_Xi_dot = a*Delta_Xi;
    Delta_Xi_dot_dot = b*Delta_Xi;
    % calcul de Delta_psi

    Delta_psi_0 = zeros(6,1);

    CI = Delta_psi_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) Delta_Xi(:,it);

    Delta_psi_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
     % calcul de Delta_eta
    Theta = var_theta(1:3,1);
    D = var_theta(4:6,1);
    
    Delta_eta_0 = a*T_SE3_m1_(Theta,D)*Delta_psi_0;
% %     Delta_eta_0 = [[a*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_0(4:6)),a*R_0']]*Delta_psi_0;
%     Delta_eta_0 = [[a*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_0(4:6)),a*eye(3)]]*Delta_psi_0;

    CI = Delta_eta_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) -ad_(Delta_Xi(:,it))*eta_X(:,it)+ Delta_Xi_dot(:,it);

    Delta_eta_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    % calcul de Delta_eta_dot
    
    Theta = var_theta(1:3,1);
    D = var_theta(4:6,1);
    Delta_eta_dot_0 = b*T_SE3_m1_(Theta,D)*Delta_psi_0;
% %     Delta_eta_dot_0 = [[b*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_dot_0(4:6)+hat_(eta_0(1:3))*eta_0(4:6))+a*hat_(eta_0(4:6))*T_SO3_m1_(Theta)-hat_(eta_0(1:3))*hat_(eta_0(4:6)),b*R_0'-a*hat_(eta_0(1:3))*R_0']]*Delta_psi_0;
%     Delta_eta_dot_0 = [[b*T_SO3_m1_(Theta),zeros(3)];[hat_(eta_dot_0(4:6)+hat_(eta_0(1:3))*eta_0(4:6))+a*hat_(eta_0(4:6))*T_SO3_m1_(Theta)-hat_(eta_0(1:3))*hat_(eta_0(4:6)),b*eye(3)-a*hat_(eta_0(1:3))]]*Delta_psi_0;

    CI = Delta_eta_dot_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) -ad_(Xi_dot(:,it))*Delta_eta_X(:,it) -ad_(Delta_Xi_dot(:,it))*eta_X(:,it)-ad_(Delta_Xi(:,it))*eta_dot_X(:,it)+ Delta_Xi_dot_dot(:,it);

    Delta_eta_dot_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    % calcul de Delta_Lambda
    M_cal = Const.M_cal;

    for it_x = 1:N_noeuds
        R = quaternion_to_matrice(QX(:,it_x));
        Delta_F_bar(:,it_x)= [zeros(3,1);hat_(Delta_psi_X(:,it_x))'*R'*[0;   0; -Const.rho*Aire*Gamma_g]];
    end
    Delta_F_bar(:,end)= [zeros(3,1);hat_(Delta_psi_X(:,it_x))'*R'*Const.F1(4:6)];

    CI = Delta_F_bar(:,end);
    CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];

    f_A = @(it) ad_(Xi(:,it))';
    f_B = @(it) M_cal*Delta_eta_dot_X(:,it) - ad_(Delta_eta_X(:,it))'*M_cal*eta_X(:,it) - ad_(eta_X(:,it))'*M_cal*Delta_eta_X(:,it) - Delta_F_bar(:,it) + ad_(Delta_Xi(:,it))'*Lambda_X(:,it);

    Delta_Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

    Delta_F0 = -Delta_Lambda_X(:,1);

    % calcul de Q_a

    CI =  zeros(Const.dim_base,1);
    CL_ind=[];
    for i=1:Const.dim_base
        CL_ind=[CL_ind,i*N_noeuds];
    end

    f_A = @(it) zeros(Const.dim_base,Const.dim_base);
    f_B = @(it)  -Phi(:,:,it)*Const.B'*Delta_Lambda_X(:,it);

    Delta_Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

    Delta_Qa = -Delta_Qa_X(:,1);
    
    J0e = [J0e,Delta_F0];
    Jee = [Jee,Delta_Qa];
end

J = [J00,J0e;Je0,Jee];
% J = (J+J')/2;
% J = Jee;
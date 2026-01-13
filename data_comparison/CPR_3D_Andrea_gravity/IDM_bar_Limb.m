function [Var,F,Lambda,Bar_Res_r_i,Bar_Res_e_i] = IDM_bar_Limb(Config,Var,i_limb)

% Params spectral
N_noeuds = Config.N_noeuds; % nombre de noeuds
[DX,X_grille]=cheb(N_noeuds-1,Config.robot.limb.l(i_limb));  % sur une grille [0,L]


Config.robot.limb.i_limb = i_limb;

% entrées

qr = Var.q(i_limb);
qr_dot = Config.simu.dyn*Var.q_dot(i_limb);
qr_dot_dot = Config.simu.dyn*Var.q_dot_dot(i_limb);
qe = Var.q(Config.robot.nl + 1 + (i_limb-1)*Config.robot.limb.dim_qe_i:Config.robot.nl + i_limb*Config.robot.limb.dim_qe_i);
qe_dot = Config.simu.dyn*Var.q_dot(Config.robot.nl + 1 + (i_limb-1)*Config.robot.limb.dim_qe_i:Config.robot.nl + i_limb*Config.robot.limb.dim_qe_i);
qe_dot_dot = Config.simu.dyn*Var.q_dot_dot(Config.robot.nl + 1 + (i_limb-1)*Config.robot.limb.dim_qe_i:Config.robot.nl + i_limb*Config.robot.limb.dim_qe_i);

% inital value k = 0
g_k = [Config.robot.limb.Roi{i_limb} Config.robot.limb.roi{i_limb};0 0 0 1];
eta_k = zeros(6,1);
eta_k_dot =  zeros(6,1);

% Forward

for j = 1:3
    if j==1     % Rigid joint j=1 
        gj_k= [cos(qr) -sin(qr)  0 0; sin(qr) cos(qr)  0 0;  0 0 1 0;0 0 0 1];
        eta_j_k = Config.robot.limb.Aoi{i_limb}*qr_dot;
        eta_j_k_dot = Config.robot.limb.Aoi{i_limb}*qr_dot_dot;
    end
    if j==2    % Soft joint j = 2

        for it_x = 1:N_noeuds
            Phi(:,:,it_x) = Base_Phi(X_grille(it_x),Config);
            Xi_a = Phi(:,:,it_x)'*qe;
            Xi_a_dot = Phi(:,:,it_x)'*qe_dot;
            Xi_a_dot_dot = Phi(:,:,it_x)'*qe_dot_dot;

            Xi(:,it_x) = Config.B*Xi_a + Config.Xi_0{i_limb};
            Xi_dot(:,it_x) = Config.B*Xi_a_dot;
            Xi_dot_dot(:,it_x) = Config.B*Xi_a_dot_dot;
        end
            
        % Calcul de Quaternion
        
        CI = [1;0;0;0];
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];
        
        f_A = @(it) quaternion_dot2(Xi(:,it));
        f_B = @(it) zeros(max(size(CI)),1);
        
        Quat_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Var.Quat_X{i_limb}=Quat_X;
        % calcul de r
        
        CI = [0;0;0];
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];
        
        f_A = @(it) zeros(max(size(CI)),max(size(CI)));
        f_B = @(it) r_dot(Quat_X(:,it),Xi(4:6,it));
        
        r_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Var.r_X{i_limb}=r_X;
    
        % calcul de eta

        CI = zeros(6,1);
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];
    
        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Xi_dot(:,it);
    
        eta_j_k = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        
        % calcul de eta_dot
    
        CI = zeros(6,1);
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];
    
        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Xi_dot_dot(:,it)- ad_(Xi_dot(:,it))*eta_j_k(:,it);
    
        eta_j_k_dot = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    end
    if j==3    % Leaf joint j = 3
        gj_k = [Config.robot.limb.R3i{i_limb},Config.robot.limb.r3i{i_limb};0 0 0 1];
        eta_j_k = zeros(6,1);
        eta_j_k_dot = zeros(6,1);
    end
    if j==2
        for it_grille =1:N_noeuds
            gj_k = [quaternion_to_matrice(Quat_X(:,it_grille)),r_X(:,it_grille);0 0 0 1];
            g_j = g_k*gj_k;
            gk_j = inv(gj_k);
            eta_j(:,it_grille) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k+eta_j_k(:,it_grille);
            eta_j_dot(:,it_grille) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k_dot+ad_(eta_j(:,it_grille))*eta_j_k(:,it_grille)+eta_j_k_dot(:,it_grille); 
        end
    else
        g_j = g_k*gj_k;
        gk_j = inv(gj_k);

        eta_j = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k+eta_j_k;
        eta_j_dot = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k_dot+ad_(eta_j)*eta_j_k+eta_j_k_dot;
    end
    Var.g_j{i_limb}{j} = g_j;
    Var.eta_j{i_limb}{j} = eta_j;
    Var.eta_j_dot{i_limb}{j} = eta_j_dot;
    Var.gj_k{i_limb}{j} = gj_k;
    Var.eta_j_k{i_limb}{j} = eta_j_k;
    Var.eta_j_k_dot{i_limb}{j} = eta_j_k_dot;

    g_k = g_j;
    eta_k = eta_j(:,end);
    eta_k_dot = eta_j_dot(:,end);
end

% Backward

for j = 3:-1:1
    if j == 3
        Fl_j = zeros(6,1);
        Fext_j = -Var.Fi_p{i_limb};
    end
    if j==2
        gj_l = inv(Var.gj_k{i_limb}{j+1});
        Fl_j = Ad_g_(gj_l(1:3,1:3),gj_l(1:3,4))'*Fl;
        Fext_j = zeros(6,1);
    end
    if j==1
        CI = -Fl;
        CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];
        
        M_cal_l = Config.robot.limb.M_cal_l;
        [F0,F1,F_Bar] = Forces_exterieur_Limb(Config,Var,i_limb);

        eta_j = Var.eta_j{i_limb}{j+1};
        eta_j_dot = Var.eta_j_dot{i_limb}{j+1} ;
    
        f_A = @(it) ad_(Xi(:,it))';
        f_B = @(it) M_cal_l*eta_j_dot(:,it) - ad_(eta_j(:,it))'*M_cal_l*eta_j(:,it) - F_Bar(:,it);
        
        Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        
        Var.Lambda_X{i_limb} = Lambda_X;

        Fl_j = -Lambda_X(:,1);
    
        % calcul de Q_a
    
        CI =  zeros(Config.robot.limb.dim_qe_i,1);
        CL_ind=[];
        for i=1:Config.robot.limb.dim_qe_i
            CL_ind=[CL_ind,i*N_noeuds];
        end
    
        f_A = @(it) zeros(Config.robot.limb.dim_qe_i,Config.robot.limb.dim_qe_i);
        f_B = @(it)  -Phi(:,:,it)*Config.B'*(Lambda_X(:,it)-Config.robot.limb.H_cal * Xi(:,it));
    
        Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
        Q_a = Qa_X(:,1);
    end
    Fj = Fl_j - Fext_j;
    
    Var.Fj{i_limb}{j} = Fj;
    Var.Fl_j{i_limb}{j} = Fl_j;
    
    Fl = Fj;

end
F = Var.Fj{i_limb}{1};
Lambda = Var.Lambda_X{i_limb};


% if Var.t< 100*Config.dt
%     Var.tau{1} = 1*(10*Config.dt) + 0.2*sin(4*pi* (10*Config.dt));
%     Var.tau{2} = 1*(10*Config.dt) + 0.2*sin(3*pi* (30*Config.dt));
%     Var.tau{3} = 1*(10*Config.dt) + 0.2*sin(4*pi* (30*Config.dt));
% end
% if Var.t< 80*Config.dt
%     Var.tau{1} = 1*(10*Config.dt) + 0.2*sin(4*pi* (10*Config.dt));
%     Var.tau{2} = 1*(10*Config.dt) + 0.2*sin(3*pi* (30*Config.dt));
%     Var.tau{3} = 1*(10*Config.dt) + 0.2*sin(4*pi* (Var.t-50*Config.dt));
% end
% if Var.t< 50*Config.dt
%     Var.tau{1} = 1*(10*Config.dt) + 0.2*sin(4*pi* (10*Config.dt));
%     Var.tau{2} = 1*(10*Config.dt) + 0.2*sin(3*pi* (Var.t-20*Config.dt));
%     Var.tau{3} = 1*(10*Config.dt);
% end
% if Var.t< 20*Config.dt
%     Var.tau{1} = 1*(10*Config.dt) + 0.2*sin(4*pi* (Var.t-10*Config.dt));
%     Var.tau{2} = 1*(10*Config.dt);
%     Var.tau{3} = 1*(10*Config.dt);
% end
% if Var.t< 10*Config.dt
%     Var.tau{1} = 1*(Var.t-Config.dt);
%     Var.tau{2} = 1*(Var.t-Config.dt);
%     Var.tau{3} = 1*(Var.t-Config.dt);
% end

% Var.qoi_d = Config.robot.limb.qoi_0 + (Config.robot.limb.qoi_f-Config.robot.limb.qoi_0)*Var.t;
% if Var.t<= 0
%     Var.qoi_d(i_limb) = qr;
%     Var.tau{i_limb} = 0;
% end
% 
% Var.tau{i_limb} = Var.tau{i_limb} - 1e-1*(qr - Var.qoi_d(i_limb));

Bar_Res_r_i = Config.robot.limb.Aoi{i_limb}'*F-Var.tau(i_limb);
Bar_Res_e_i = -Q_a;

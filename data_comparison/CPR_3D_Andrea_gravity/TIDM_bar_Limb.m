function [Var,Delta_F,Delta_Lambda,Delta_Bar_Res_r_i,Delta_Bar_Res_e_i] = TIDM_bar_Limb(Config,Var,i_limb)

a = Config.Integrateur.newmark.a;
b = Config.Integrateur.newmark.b;

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


Delta_qr = Var.Delta_q(i_limb);
Delta_qr_dot = Config.simu.dyn*a*Delta_qr;
Delta_qr_dot_dot = Config.simu.dyn*b*Delta_qr;
Delta_qe = Var.Delta_q(Config.robot.nl + 1 + (i_limb-1)*Config.robot.limb.dim_qe_i:Config.robot.nl + i_limb*Config.robot.limb.dim_qe_i);
Delta_qe_dot = Config.simu.dyn*a*Delta_qe;
Delta_qe_dot_dot = Config.simu.dyn*b*Delta_qe;

% inital value k = 0
g_k = [Config.robot.limb.Roi{i_limb} Config.robot.limb.roi{i_limb};0 0 0 1];
eta_k = zeros(6,1);
eta_k_dot =  zeros(6,1);
Delta_zeta_k = zeros(6,1);
Delta_eta_k = zeros(6,1);
Delta_eta_k_dot =  zeros(6,1);

%----------------------------%

% Forward

for j = 1:3
    if j==1     % Rigid joint j=1 
        Delta_zeta_j_k = Config.robot.limb.Aoi{i_limb} * Delta_qr;
        Delta_eta_j_k = Config.robot.limb.Aoi{i_limb}*Delta_qr_dot;
        Delta_eta_j_k_dot = Config.robot.limb.Aoi{i_limb}*Delta_qr_dot_dot;
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

            Delta_Xi(:,it_x) = Config.B*Phi(:,:,it_x)'*Delta_qe;
            Delta_Xi_dot(:,it_x) = Config.B*Phi(:,:,it_x)'*Delta_qe_dot;
            Delta_Xi_dot_dot(:,it_x) = Config.B*Phi(:,:,it_x)'*Delta_qe_dot_dot;

        end
        
        eta_j_k = Var.eta_j_k{i_limb}{j};
        eta_j_k_dot = Var.eta_j_k_dot{i_limb}{j};

        % Calcul de delta Delta_zeta_j_k
        
        CI = zeros(6,1);
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];
    
        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Delta_Xi(:,it);
    
        Delta_zeta_j_k = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
        % calcul de Delta_eta_j_k

        CI = zeros(6,1);
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];
    
        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Delta_Xi_dot(:,it)- ad_(Delta_Xi(:,it))*eta_j_k(:,it);
    
        Delta_eta_j_k = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        
        % calcul de Delta_eta_j_k_dot
    
        CI = zeros(6,1);
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];    

        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Delta_Xi_dot_dot(:,it)- ad_(Delta_Xi(:,it))*eta_j_k_dot(:,it) - ad_(Xi_dot(:,it))*Delta_eta_j_k(:,it) - ad_(Delta_Xi_dot(:,it))*eta_j_k(:,it);
    
        Delta_eta_j_k_dot = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    end
    if j==3    % Leaf joint j = 3
        Delta_zeta_j_k = zeros(6,1);
        Delta_eta_j_k = zeros(6,1);
        Delta_eta_j_k_dot = zeros(6,1);
    end
    if j==2
        for it_grille =1:N_noeuds
            
            gj_k = [quaternion_to_matrice(Var.Quat_X{i_limb}(:,it_grille)),Var.r_X{i_limb}(:,it_grille);0 0 0 1];
            gk_j = inv(gj_k);
            
            Delta_zeta_j(:,it_grille) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_zeta_k + Delta_zeta_j_k(:,it_grille);

            Delta_eta_j(:,it_grille) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_eta_k -ad_(Delta_zeta_j_k(:,it_grille))*Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k + Delta_eta_j_k(:,it_grille);
            Delta_eta_j_dot(:,it_grille) = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_eta_k_dot- ad_(Delta_zeta_j_k(:,it_grille))*Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k_dot+ad_(eta_j)*Delta_eta_j_k(:,it_grille)+ad_(Delta_eta_j)*eta_j_k(:,it_grille)+Delta_eta_j_k_dot(:,it_grille);
        end
    else
        gj_k = Var.gj_k{i_limb}{j};
        eta_j =  Var.eta_j{i_limb}{j};
        eta_j_k =  Var.eta_j_k{i_limb}{j};
        gk_j = inv(gj_k);

        Delta_zeta_j = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_zeta_k + Delta_zeta_j_k;
        Delta_eta_j = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_eta_k -ad_(Delta_zeta_j_k)*Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k + Delta_eta_j_k;
        Delta_eta_j_dot = Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*Delta_eta_k_dot- ad_(Delta_zeta_j_k)*Ad_g_(gk_j(1:3,1:3),gk_j(1:3,4))*eta_k_dot+ad_(eta_j)*Delta_eta_j_k+ad_(Delta_eta_j)*eta_j_k+Delta_eta_j_k_dot;
    end
    Var.Delta_zeta_j{i_limb}{j} = Delta_zeta_j;
    Var.Delta_eta_j{i_limb}{j} = Delta_eta_j;
    Var.Delta_eta_j_dot{i_limb}{j} = Delta_eta_j_dot;
    
    Var.Delta_zeta_j_k{i_limb}{j} = Delta_zeta_j_k;
    Var.Delta_eta_j_k{i_limb}{j} = Delta_eta_j_k;
    Var.Delta_eta_j_k_dot{i_limb}{j} = Delta_eta_j_k_dot;

    Delta_zeta_k = Delta_zeta_j(:,end);
    Delta_eta_k = Delta_eta_j(:,end);
    Delta_eta_k_dot = Delta_eta_j_dot(:,end);

    eta_k = Var.eta_j{i_limb}{j};
    eta_k_dot = Var.eta_j_dot{i_limb}{j}(:,end);
end

% Backward

for j = 3:-1:1
    if j == 3
        Delta_Fl_j = zeros(6,1);
        Delta_Fext_j = -Var.Delta_Fi_p{i_limb};
    end
    if j==2
        Delta_zeta_j = Var.Delta_zeta_j{i_limb}{j+1}(:,end);
        Delta_zeta_j_k = Var.Delta_zeta_j_k{i_limb}{j+1}(:,end);;
        gj_l = inv(Var.gj_k{i_limb}{j+1});
        Fl = Var.Fj{i_limb}{j+1};

        Delta_Fl_j = Ad_g_(gj_l(1:3,1:3),gj_l(1:3,4))'*(Delta_Fl-ad_(Delta_zeta_j)'*Fl);
        Delta_Fl_j = Ad_g_(gj_l(1:3,1:3),gj_l(1:3,4))'*(Delta_Fl-ad_(Delta_zeta_j_k)'*Fl);
        Delta_Fext_j = zeros(6,1);
    end
    if j==1
        CI = -Delta_Fl;
        CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];
        
        M_cal_l = Config.robot.limb.M_cal_l;
        [F0,F1,F_Bar] = Forces_exterieur_Limb(Config,Var,i_limb);
        [Delta_F0,Delta_F1,Delta_F_Bar]  = Delta_Forces_exterieur_Limb(Config,Var,i_limb) ;

        eta_j = Var.eta_j{i_limb}{j+1};
        eta_j_dot = Var.eta_j_dot{i_limb}{j+1} ;
        Delta_eta_j = Var.Delta_eta_j{i_limb}{j+1} ;
        Delta_eta_j_dot = Var.Delta_eta_j_dot{i_limb}{j+1} ;
        Lambda_X = Var.Lambda_X{i_limb};
        
        f_A = @(it) ad_(Xi(:,it))';
        f_B = @(it) M_cal_l*Delta_eta_j_dot(:,it) - ad_(Delta_eta_j(:,it))'*M_cal_l*eta_j(:,it) -  ad_(eta_j(:,it))'*M_cal_l*Delta_eta_j(:,it)+ ad_(Delta_Xi(:,it))'*Lambda_X(:,it) -Delta_F_Bar(:,it);
        
        Delta_Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        
        Var.Delta_Lambda_X{i_limb} = Delta_Lambda_X;

        Delta_Fl_j = -Delta_Lambda_X(:,1);
    
        % calcul de Q_a
    
        CI =  zeros(Config.robot.limb.dim_qe_i,1);
        CL_ind=[];
        for i=1:Config.robot.limb.dim_qe_i
            CL_ind=[CL_ind,i*N_noeuds];
        end
    
        f_A = @(it) zeros(Config.robot.limb.dim_qe_i,Config.robot.limb.dim_qe_i);
        f_B = @(it)  -Phi(:,:,it)*Config.B'*(Delta_Lambda_X(:,it)-Config.robot.limb.H_cal * Delta_Xi(:,it));
    
        Delta_Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
        Delta_Q_a = Delta_Qa_X(:,1);
    end
    Delta_Fj = Delta_Fl_j - Delta_Fext_j;
    
    Var.Delta_Fj{i_limb}{j} = Delta_Fj;
    Var.Delta_Fl_j{i_limb}{j} = Delta_Fl_j;
    
    Delta_Fl = Delta_Fj;

end
Delta_F = Var.Delta_Fj{i_limb}{1};
Delta_Lambda = Var.Delta_Lambda_X{i_limb};

Delta_Bar_Res_r_i = Config.robot.limb.Aoi{i_limb}'*Delta_F;
Delta_Bar_Res_e_i = -Delta_Q_a;
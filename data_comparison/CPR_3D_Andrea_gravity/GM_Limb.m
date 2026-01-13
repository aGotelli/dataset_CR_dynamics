function   [g1,g2,g3,Q2X,r2X] = GM_Limb(i_limb,Config,Var)
    %%
    % Params spectral
    N_noeuds = Config.N_noeuds; % nombre de noeuds
    [DX,X_grille]=cheb(N_noeuds-1,Config.robot.limb.l(i_limb));  % sur une grille [0,L]
    Config.robot.limb.i_limb = i_limb;
    
    % entrées
    
    qr = Var.q(i_limb);
    qe = Var.q(Config.robot.nl + 1 + (i_limb-1)*Config.robot.limb.dim_qe_i:Config.robot.nl + i_limb*Config.robot.limb.dim_qe_i);
    
    g0 = [Config.robot.limb.Roi{i_limb} Config.robot.limb.roi{i_limb};0 0 0 1];
    
    % Forward geo gj_k
    % Rigid joint j=1 
    g1_0 = [cos(qr) -sin(qr)  0 0; sin(qr) cos(qr)  0 0;  0 0 1 0;0 0 0 1];
    g1 = g0*g1_0;
    
    % Soft joint j = 2
    
    for it_x = 1:N_noeuds
        Phi(:,:,it_x) = Base_Phi(X_grille(it_x),Config);
        Xi_a = Phi(:,:,it_x)'*qe;
        Xi(:,it_x) = Config.B*Xi_a +  Config.Xi_0{i_limb};
    end
        % Calcul de Quaternion
    
    CI = [1;0;0;0];
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];
    
    f_A = @(it) quaternion_dot2(Xi(:,it));
    f_B = @(it) zeros(max(size(CI)),1);
    
    Q2X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    % calcul de r
    
    CI = [0;0;0];
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];
    
    f_A = @(it) zeros(max(size(CI)),max(size(CI)));
    f_B = @(it) r_dot(Q2X(:,it),Xi(4:6,it));
    
    r2X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    
    g2_1 = [quaternion_to_matrice(Q2X(:,end)),r2X(:,end);0 0 0 1];
    g2 = g1*g2_1;
    
    % Leaf joint j = 3
    g3_2 = [Config.robot.limb.R3i{i_limb},Config.robot.limb.r3i{i_limb};0 0 0 1];
    g3 = g2*g3_2;
end
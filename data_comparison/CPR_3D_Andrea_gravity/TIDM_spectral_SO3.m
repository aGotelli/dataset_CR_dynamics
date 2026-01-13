function [RES2,J2] = TIDM_spectral_SO3(q_0,r_0,eta_0,eta_dot_0,time,Const,Config)

a_pen = 1;

% Gain intégrateur de Newmark
Beta  = 1/6; % interpolation 1/4; % valeur moyenne 1/6
Gamma = 1/2;
h = Config.dt;
a = Gamma/(Beta*h);
b = 1/(Beta*h^2);

Theta_d = Config.Theta_d(:,time) ;
r_d = Config.r_d(:,time);

J = [];
J2 = [];
Jg = [];

% Définition des noeuds
N_noeuds = Config.N_noeuds; % nombre de noeuds

X_grille_T = [];
Xi_T = [];
Xi_dot_T = [];
Xi_dot_dot_T = [];
QX_T = [];
rX_T = [];
eta_X_T = [];
eta_dot_X_T = [];
Lambda_X_T = [];
Q_a_T = [];
lambda = Const.lambda;
Delta_eta_0 = zeros(6,1);
Delta_eta_dot_0 = zeros(6,1);

for it_troncon = 1:Config.nb_poutre

    [DX,X_grille]=cheb(N_noeuds-1,Config.Li(it_troncon));  % sur une grille [0,L]
    X_grille_T = [X_grille_T,X_grille'];
    %calcul des Xi, Xi_dot et Xi_dot_dot sur les noeuds
    for it_x = 1:N_noeuds
        Phi(:,:,it_x) = Base_Phi(X_grille(it_x),time,Const,Config);
        Xi_a = Phi(:,:,it_x)'*Const.q(1+(it_troncon-1)*Const.dim_base_i:it_troncon*Const.dim_base_i);
        Xi_dot_a = Phi(:,:,it_x)'*Const.q_dot(1+(it_troncon-1)*Const.dim_base_i:it_troncon*Const.dim_base_i);
        Xi_dot_dot_a = Phi(:,:,it_x)'*Const.q_dot_dot(1+(it_troncon-1)*Const.dim_base_i:it_troncon*Const.dim_base_i);

        Xi(:,it_x) = Const.B*Xi_a +  Const.B_bar*Const.Xi_c + Const.B*Const.Xi_0;
        Xi_dot(:,it_x) = Const.B*Xi_dot_a;
        Xi_dot_dot(:,it_x) = Const.B*Xi_dot_dot_a;
    
    end
    Xi_T = [Xi_T,Xi];
    Xi_dot_T = [Xi_dot_T,Xi_dot];
    Xi_dot_dot_T = [Xi_dot_dot_T,Xi_dot_dot];
    % Calcul de Quaternion

    CI = q_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];

    f_A = @(it) quaternion_dot2(Xi(:,it));
    f_B = @(it) zeros(max(size(CI)),1);

    QX = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    QX_T = [QX_T,QX];
    q_0 = QX(:,end);

    % calcul de r

    CI = r_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1];

    f_A = @(it) zeros(max(size(CI)),max(size(CI)));
    f_B = @(it) r_dot(QX(:,it),Xi(4:6,it));

    rX = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    rX_T = [rX_T,rX];
    r_0 = rX(:,end);

    % calcul de eta

    CI = eta_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) Xi_dot(:,it);

    eta_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    eta_X_T = [eta_X_T,eta_X];
    eta_0 = eta_X(:,end);
    
    % calcul de eta_dot

    CI = eta_dot_0;
    CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

    f_A = @(it) -ad_(Xi(:,it));
    f_B = @(it) Xi_dot_dot(:,it)- ad_(Xi_dot(:,it))*eta_X(:,it);

    eta_dot_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    eta_dot_X_T = [eta_dot_X_T,eta_dot_X];
    eta_dot_0 = eta_dot_X(:,end);
end

 figure(10);
%     hold off
    for it_trace = 1:Config.nb_poutre
        a_trace = (it_trace-1)*Config.N_noeuds+1;
        b_trace = it_trace*Config.N_noeuds;
%         plot(rX(1,a_trace:b_trace),rX(3,a_trace:b_trace),'r','LineWidth',2)
% hold on
        plot3(rX(1,a_trace:b_trace),rX(2,a_trace:b_trace),rX(3,a_trace:b_trace),'y','LineWidth',2)
        hold on
        for it_R = a_trace:b_trace
            Rm = quaternion_to_matrice(QX_T(:,it_R));
            plot3([rX(1,it_R),rX(1,it_R)+Rm(1,1)],[rX(2,it_R),rX(2,it_R)+Rm(2,1)],[rX(3,it_R),rX(3,it_R)+Rm(3,1)],'b','LineWidth',2)
            plot3([rX(1,it_R),rX(1,it_R)+Rm(1,2)],[rX(2,it_R),rX(2,it_R)+Rm(2,2)],[rX(3,it_R),rX(3,it_R)+Rm(3,2)],'r','LineWidth',2)
            plot3([rX(1,it_R),rX(1,it_R)+Rm(1,3)],[rX(2,it_R),rX(2,it_R)+Rm(2,3)],[rX(3,it_R),rX(3,it_R)+Rm(3,3)],'g','LineWidth',2)
        end
%         plot3(Config.L*ones(size(rX(1,a_trace:b_trace))),rX(2,a_trace:b_trace),rX(3,a_trace:b_trace),'LineWidth',2,'Color',[0.8,0.8,0.8])
%         plot3(rX(1,a_trace:b_trace),Config.L*ones(size(rX(2,a_trace:b_trace))),rX(3,a_trace:b_trace),'LineWidth',2,'Color',[0.8,0.8,0.8])
%         plot3(rX(1,a_trace:b_trace),rX(2,a_trace:b_trace),0*ones(size(rX(3,a_trace:b_trace))),'LineWidth',2,'Color',[0.8,0.8,0.8])
hold off
    end
    grid on
    axis equal
    xlim([-0.6*Config.L 1.1*Config.L])
    ylim([-0.35*Config.L 0.35*Config.L])
    zlim([-0.35*Config.L 0.35*Config.L])
    titre = ['\theta_d = ' num2str(Config.Theta_d(1,time)*180/pi) ', r_d = ' num2str(Config.r_d(1,time))];
    title(titre);

    drawnow
    
for it_troncon = Config.nb_poutre:-1:1
    % calcul de Lambda
    M_cal = Const.M_cal;
    
    X_grille = X_grille_T(1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds)';
    QX = QX_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    rX = rX_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    eta_X = eta_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    eta_dot_X = eta_dot_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);

    Xi = Xi_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    Xi_dot = Xi_dot_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    Xi_dot_dot = Xi_dot_dot_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
    
    Const.it_troncon = it_troncon;
    for it_x = 1:N_noeuds
        [F_0,F_1,F_bar(:,it_x)] = Forces_exterieur(X_grille(it_x),QX(:,it_x),rX(:,it_x),0,Const,Config);
    end
    figure(10);
    hold on
    for it_trace = 1:Config.nb_poutre
            a_trace = (it_trace-1)*Config.N_noeuds+1;
            b_trace = it_trace*Config.N_noeuds;
            for it_R = a_trace:b_trace
                Rm = quaternion_to_matrice(QX_T(:,it_R))*F_bar(4:6,it_R);
                Rm =Rm/norm(Rm);
                plot3([rX(1,it_R),rX(1,it_R)+Rm(1,1)],[rX(2,it_R),rX(2,it_R)+Rm(2,1)],[rX(3,it_R),rX(3,it_R)+Rm(3,1)],'c','LineWidth',2)
          end
    end
    hold off
    drawnow
    
    if it_troncon == Config.nb_poutre
        Rm = quaternion_to_matrice(QX_T(:,end));
        Rd = Exp_SO3_(Theta_d);
        gm = [Rm,rX_T(:,end);0 0 0 1];
        gd = [Rd,r_d;0 0 0 1];

        dgm = inv(gd)*gm;
        Theta = Log_SO3_(dgm(1:3,1:3));
        d = rX_T(:,end)-r_d;
        J_lambda = [T_SO3_m1_(Theta),zeros(3,3);zeros(3,3),Rm];
        F_1 = F_1 - J_lambda'*(lambda + a_pen*Const.EI*[Theta;d]) ;
    end
    figure(10);
    hold on
    for it_trace = 1:Config.nb_poutre
            it_R = it_trace*Config.N_noeuds;
            Rm = quaternion_to_matrice(QX_T(:,end));
            Rm = F_1(1:3);
            Rm =Rm/norm(Rm);
            plot3([rX(1,it_R),rX(1,it_R)+Rm(1,1)],[rX(2,it_R),rX(2,it_R)+Rm(2,1)],[rX(3,it_R),rX(3,it_R)+Rm(3,1)],'m','LineWidth',2)
            Rm = quaternion_to_matrice(QX_T(:,end));
            Rm = Rm*F_1(4:6);
            Rm =Rm/norm(Rm);
            plot3([rX(1,it_R),rX(1,it_R)+Rm(1,1)],[rX(2,it_R),rX(2,it_R)+Rm(2,1)],[rX(3,it_R),rX(3,it_R)+Rm(3,1)],'k','LineWidth',2)
    end
    hold off
    drawnow
    CI = F_1;
    CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];

    f_A = @(it) ad_(Xi(:,it))';
    f_B = @(it) M_cal*eta_dot_X(:,it) - ad_(eta_X(:,it))'*M_cal*eta_X(:,it) - F_bar(:,it);

%     f_B = @(it) -F_bar(:,it);

    Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
    F_1 = Lambda_X(:,1);
    Lambda_X_T = [Lambda_X,Lambda_X_T];

    % calcul de Q_a

    CI =  zeros(Const.dim_base_i,1);
    CL_ind=[];
    for i=1:Const.dim_base_i
        CL_ind=[CL_ind,i*N_noeuds];
    end

    f_A = @(it) zeros(Const.dim_base_i,Const.dim_base_i);
    f_B = @(it)  -Phi(:,:,it)*Const.B'*Lambda_X(:,it);

    Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

    Q_a = -Qa_X(:,1);
    Q_a_T = [Q_a;Q_a_T];
end
%theta

Rm = quaternion_to_matrice(QX_T(:,end));
Rd = Exp_SO3_(Theta_d);
gm = [Rm,rX_T(:,end);0 0 0 1];
gd = [Rd,r_d;0 0 0 1];

dgm = inv(gd)*gm;

Phi_R = Log_SO3_(dgm(1:3,1:3));


%d^r_m
Phi_r = rX_T(:,end)-r_d;
       
RES2 = [Q_a_T;Phi_R;Phi_r];
% Partie Variations

% on construit la matrice jacobienne J

Aire = pi*Const.Rc^2;
Gamma_g = Const.Gamma_g;

%variation sur Delta_q 

for it_D_q = 1:Const.dim_base + 6

    Delta_psi_X_T = [];
    Delta_Qa_T = [];
    Delta_Xi_T = [];
    Delta_eta_X_T = [];
    Delta_eta_dot_X_T = [];
    
    Delta_q_T = zeros(Const.dim_base,1);
    Delta_lamba = zeros(6,1);
    
    if it_D_q <= Const.dim_base
        Delta_q_T(it_D_q,1) = 1;
    else
        Delta_lamba(it_D_q-Const.dim_base,1) = 1;
    end
    Delta_psi_0 = zeros(6,1);
    Delta_eta_0 = zeros(6,1);
    Delta_eta_dot_0 = zeros(6,1);
    
    for it_troncon = 1:Config.nb_poutre
        X_grille = X_grille_T(1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds)';

        Delta_q = Delta_q_T(1+(it_troncon-1)*Const.dim_base_i:it_troncon*Const.dim_base_i,1);
        Xi = Xi_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);

        for it_x = 1:N_noeuds
            Phi(:,:,it_x) = Base_Phi(X_grille(it_x),time,Const,Config);
            Delta_Xi(:,it_x) = Const.B*Phi(:,:,it_x)'*Delta_q;     
        end
        Delta_Xi_dot = a*Delta_Xi;
        Delta_Xi_dot_dot = b*Delta_Xi;
        
        Delta_Xi_T = [Delta_Xi_T,Delta_Xi];

        % calcul de Delta_psi

        CI = Delta_psi_0;
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) Delta_Xi(:,it);

        Delta_psi_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Delta_psi_X_T = [Delta_psi_X_T,Delta_psi_X];
        Delta_psi_0 = Delta_psi_X(:,end);
        
        % calcul de Delta_eta

        CI = Delta_eta_0;
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) -ad_(Delta_Xi(:,it))*eta_X(:,it)+ Delta_Xi_dot(:,it);

        Delta_eta_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Delta_eta_X_T = [Delta_eta_X_T,Delta_eta_X];
        Delta_eta_0 = Delta_eta_X(:,end);
        
    % calcul de Delta_eta_dot

        CI = Delta_eta_dot_0;
        CL_ind=[1,N_noeuds+1,2*N_noeuds+1,3*N_noeuds+1,4*N_noeuds+1,5*N_noeuds+1];

        f_A = @(it) -ad_(Xi(:,it));
        f_B = @(it) -ad_(Xi_dot(:,it))*Delta_eta_X(:,it) -ad_(Delta_Xi_dot(:,it))*eta_X(:,it)-ad_(Delta_Xi(:,it))*eta_dot_X(:,it)+ Delta_Xi_dot_dot(:,it);

        Delta_eta_dot_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Delta_eta_dot_X_T = [Delta_eta_dot_X_T,Delta_eta_dot_X];
        Delta_eta_dot_0 = Delta_eta_dot_X(:,end);
    
%         Jqv = reshape(quaternion_to_matrice(QX(:,end))*hat_(Delta_psi_0),[3*3,1]);
%         Jg = [Jg,Jqv];
    end
    
    for it_troncon = Config.nb_poutre:-1:1
        % calcul de Delta_Lambda
        M_cal = Const.M_cal;
        Const.it_troncon = it_troncon;
        X_grille = X_grille_T(1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds)';

        QX = QX_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Xi = Xi_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Delta_Xi =  Delta_Xi_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Delta_psi_X = Delta_psi_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Lambda_X = Lambda_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Delta_eta_X = Delta_eta_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
        Delta_eta_dot_X = Delta_eta_dot_X_T(:,1+(it_troncon-1)*N_noeuds:it_troncon*N_noeuds);
         
        for it_x = 1:N_noeuds
            [F_0,F_1,F_bar(:,it_x)] = Forces_exterieur(X_grille(it_x),QX(:,it_x),rX(:,it_x),0,Const,Config);
            R = quaternion_to_matrice(QX(:,it_x));
    
%             Delta_F_bar(:,it_x)= [zeros(3,1);hat_(Delta_psi_X(:,it_x))'*F_bar(4:6,it_x)];
            Delta_F_bar(:,it_x)= [0*hat_(Delta_psi_X(:,it_x))'*R',zeros(3,3);zeros(3,3),hat_(Delta_psi_X(:,it_x))'*R']*Const.Fbar_spaciale;
            Delta_F_1= 0*[zeros(3,1);hat_(Delta_psi_X(:,it_x))'*F_1(4:6)];
        end
        
        if it_troncon == Config.nb_poutre
            Rm = quaternion_to_matrice(QX_T(:,end));
            Rd = Exp_SO3_(Theta_d);
            gm = [Rm,rX_T(:,end);0 0 0 1];
            gd = [Rd,r_d;0 0 0 1];

            dgm = inv(gd)*gm;
            d = rX_T(:,end)-r_d;

            Delta_psi_fermeture = Delta_psi_X_T(:,end);
            Theta = Log_SO3_(dgm(1:3,1:3));

            Delta_Theta = T_SO3_m1_(Theta) * Delta_psi_fermeture(1:3);
            Delta_d = Rm * Delta_psi_fermeture(4:6);

            J_lambda = [T_SO3_m1_(Theta),zeros(3,3);zeros(3,3),Rm];

            delta_J_lambda = 0*[zeros(3,3),zeros(3,3);zeros(3,3),Rm*hat_(Delta_psi_fermeture(1:3))];
            Delta_F_1 = Delta_F_1 - delta_J_lambda'*(lambda + a_pen*Const.EI*[Theta;d]) - J_lambda'*(Delta_lamba+ a_pen*Const.EI*[Delta_Theta;Delta_d]) ;
        end

        CI = Delta_F_1;
        CL_ind=[N_noeuds,2*N_noeuds,3*N_noeuds,4*N_noeuds,5*N_noeuds,6*N_noeuds];

        f_A = @(it) ad_(Xi(:,it))';
        f_B = @(it) M_cal*Delta_eta_dot_X(:,it) - ad_(Delta_eta_X(:,it))'*M_cal*eta_X(:,it) - ad_(eta_X(:,it))'*M_cal*Delta_eta_X(:,it) - Delta_F_bar(:,it) + ad_(Delta_Xi(:,it))'*Lambda_X(:,it);
%         f_B = @(it) ad_(Delta_Xi(:,it))'*Lambda_X(:,it) - Delta_F_bar(:,it);

        Delta_Lambda_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);
        Delta_F_1 = Delta_Lambda_X(:,1);

        % calcul de Delta Q_a

        CI =  zeros(Const.dim_base_i,1);
        CL_ind=[];
        for i=1:Const.dim_base
            CL_ind=[CL_ind,i*N_noeuds];
        end

        f_A = @(it) zeros(Const.dim_base,Const.dim_base);
        f_B = @(it)  -Phi(:,:,it)*Const.B'*Delta_Lambda_X(:,it);

        Delta_Qa_X = integral_spectral(f_A,f_B,CI,DX,N_noeuds,CL_ind);

        Delta_Qa = -Delta_Qa_X(:,1);
        Delta_Qa_T = [Delta_Qa;Delta_Qa_T];
    end
    Theta = Log_SO3_(dgm(1:3,1:3));

    Delta_psi_fermeture = Delta_psi_X_T(:,end);
    
    Delta_Phi_R = T_SO3_m1_(Theta) * Delta_psi_fermeture(1:3);

    Delta_Phi_r = Rm * Delta_psi_fermeture(4:6);
    J2 = [J2,[Delta_Qa_T;Delta_Phi_R;Delta_Phi_r]];
end
% val2 = max(abs(J2(Const.dim_base+1:Const.dim_base+6,:)-J2(:,Const.dim_base+1:Const.dim_base+6)'));
% J2;

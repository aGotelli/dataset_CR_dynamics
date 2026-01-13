function [T,Xi,lambda] = Time_Integration_Newton_Dynamic_CPR_3D_Spectral(Config,Var)


% Initial input

Qp(:,1) = matrice_to_quaternion(Var.Rp);
rp(:,1) = Var.rp;
eta_p(:,1) = zeros(6,1);
dot_eta_p(:,1) = zeros(6,1); 

q(:,1)         = Var.q;
q_dot(:,1)     = zeros(size(Var.q));
q_dot_dot(:,1) = zeros(size(Var.q));
a(:,1) = q_dot_dot(:,1);

lambda(:,1) = Var.lambda;
T(1) = 0;


for it_simu = 1:(Config.Nt - 1)
    clc

    Var.t = Config.time(it_simu+1);
    disp(['time = ' num2str(Var.t)])
    Var.R_p_n = quaternion_to_matrice(Qp(:,it_simu));
    Var.r_p_n = rp(:,it_simu);

    [Var.g_0,Var.eta_0,Var.eta_0_dot,Var.mu_p] = Prediction_Platform(eta_p(:,it_simu),dot_eta_p(:,it_simu),Config,Var);
    
    [Var.q,Var.q_dot,Var.q_dot_dot,Var.a] = Prediction_Limb(q(:,it_simu),q_dot(:,it_simu),q_dot_dot(:,it_simu),a(:,it_simu),Config);
    

    % if Var.t<=1
    %     Var.qoi_d = Config.robot.limb.qoi_0 + (Config.robot.limb.qoi_f1-Config.robot.limb.qoi_0)*Var.t*0;
    % else
    %     Var.qoi_d = Config.robot.limb.qoi_f1 + (Config.robot.limb.qoi_f2-Config.robot.limb.qoi_f1)*(0*Var.t-1);
    % end
    % Var.q(1:Config.robot.nl) = Var.qoi_d(1:Config.robot.nl);

    % figure(12)
    % plot(Var.t,Var.qoi_d,'*')
    % hold on
    % grid on
    
    % Var.q(1:Config.robot.nl) = Var.qoi_d;

    % while norm(Var.q(1:Config.robot.nl) - Var.qoi_d) > Config.Res_com || (build + Var.t)== 0
    %     disp([s'norm commande = ' num2str(norm(Var.q(1:Config.robot.nl) - Var.qoi_d))])
    %     if Var.t== 0
    %         Var.qoi_d = Var.q(1:Config.robot.nl);
            Var.tau = zeros(Config.robot.nl,1);
    %     else
    %         Var.tau= Var.tau - 0.1*(Var.q(1:Config.robot.nl) - Var.qoi_d);
    %     end
    % 
        [Var,Config,Bar_Res] = Calcul_Residu(Config,Var);
        size_res = max(size(Bar_Res));
        P = eye(size_res,size_res);
        T = P([1:6,13:size_res],:);
        Bar_Res = T*Bar_Res;
        
        lambda_LM = 1e-4;

        while norm(Bar_Res) > Config.Res_min
            disp(['norm RES = ' num2str(norm(Bar_Res))])
            % disp(['\lambda_L_M = ' num2str(norm(lambda_LM))])
            J = Calcul_Jacobian(Config,Var);
            J = T*J*T';
 

            Delta_nu_bar = -pinv(J)*Bar_Res;

            Delta_nu_bar = T'*Delta_nu_bar;
    
            
         
            Var_temp = Var;
            Bar_Res_temp = Bar_Res;
            
            [Var.g_0,Var.eta_0,Var.eta_0_dot,Var.mu_p] = Correction_Platform(Config,Var,Delta_nu_bar);
            
            [Var.q,Var.q_dot,Var.q_dot_dot] = Correction_Limb(Config,Var,Delta_nu_bar);
            
            for it_limb = 1 :Config.robot.nl
                it_lambda = Var.size.lambda/Config.robot.nl;
                Var.lambda{it_limb} = Var.lambda{it_limb} + Delta_nu_bar(1+Var.size.mu_p+Var.size.q +(it_limb-1)*it_lambda:Var.size.mu_p+Var.size.q +(it_limb)*it_lambda);
            end
            
            [Var,Config,Bar_Res] = Calcul_Residu(Config,Var);
            Bar_Res = T*Bar_Res;

            if(Config.plot_NR)
                figure(1)
                hold off
                trace_CPR_3D(Config,Var)
                view(142.5,30)
                title('Newton Loop')
                drawnow
            end
        end
        build = 1;
        if Var.t>=0.25
            Config.simu.dyn = 1;
        else
            Config.simu.dyn = 0;
        end

    if(Config.plot_t)
        figure(1)
        hold off
        trace_CPR_3D(Config,Var)
        view(142.5,30)
        drawnow
    end
    Var.R_0_n = Var.g_0(1:3,1:3);
    Qp(:,it_simu+1) = matrice_to_quaternion(Var.g_0(1:3,1:3));
    rp(:,it_simu+1) = Var.g_0(1:3,4);
    eta_p(:,it_simu+1) = Var.eta_0;
    dot_eta_p(:,it_simu+1) = Var.eta_0_dot; 

    q(:,it_simu+1)         = Var.q;
    q_dot(:,it_simu+1)     = Var.q_dot;
    q_dot_dot(:,it_simu+1) = Var.q_dot_dot;
    
    % alpha Methode
    rho_inf = Config.Integrateur.alpha_methode.rho_inf;
    
    alpha_m = (2*rho_inf-1)/(rho_inf+1);
    alpha_f = (rho_inf)/(rho_inf+1);
    % mise jour de a 
    a(:,it_simu+1) = Var.a + (1-alpha_f)/(1-alpha_m)*Var.q_dot_dot;
    
    lambda(:,it_simu+1) = Var.lambda;
    % lambda(:,it_simu+1) = lambda(:,1);
    if(Config.plot_t)
        frame = getframe(gcf);
        for it_frame = 1:10
            writeVideo(Var.video,frame);
        end
    end

end

Xi = [Qp;rp;eta_p;dot_eta_p;q;q_dot;q_dot_dot];
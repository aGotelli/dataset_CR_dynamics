function [T,q,q_dot,q_dot_dot, position_disks_simu] = Time_integration_Newton_beam_released_spectral(t,Const,Config)

T = t;
r_min = 1e-6;
Beta = 1/4; % interpolation 1/4; % valeur moyenne 1/6
Gamma = 1/2;


q(:,1)         = Const.q;
q_dot(:,1)     = Const.q_dot;
q_dot_dot(:,1) = Const.q_dot_dot;

q_0   = Const.q_0;
r_0   = Const.r_0;
eta_0 = Const.eta_0;
eta_dot_0 = Const.eta_dot_0;

qn_0   = Const.q_0;
rn_0   = Const.r_0;


N_time = max(size(t))-1
position_disks_simu = zeros(5, 3, N_time);

for it_t = 1:N_time
    
    time = t(it_t+1)
    
    Const.F1;
    
    Const.q     = q(:,it_t);
    Const.q_dot = q_dot(:,it_t);
    Const.q_dot_dot = q_dot_dot(:,it_t);
    
    [F_ad,Q_ad] = IDM_ORIN_spectral(q_0,r_0,eta_0,time,Const,Config);    
    Q_ad = 0*Q_ad + 0*Const.Dee*Const.q_dot + 0*Const.Kee*Const.q;
    F_ad = 0*F_ad;

    %   Include actuation
    
    h = Config.dt;
    a = Gamma/(Beta*h);
    b = 1/(Beta*h^2);
    
    q_n = q(:,it_t);
    q_dot_n = q_dot(:,it_t);
    q_dot_dot_n = q_dot_dot(:,it_t);
    
% prediction k=0 dans ce cas
    q_np1_k = q_n + h*q_dot_n +(1/2 - Beta)*h^2*q_dot_dot_n;    
    q_dot_np1_k = q_dot_n +(1 - Gamma)*h*q_dot_dot_n;  
    q_dot_dot_np1_k = 0*q_dot_dot_n;
    
    Const.q         = q_np1_k;
    Const.q_dot     = q_dot_np1_k;
    Const.q_dot_dot = q_dot_dot_np1_k;
    
    eta_0 = Const.eta_0;
    eta_dot_0 = Const.eta_dot_0;
    
    Const.eta_0 = eta_0;
    Const.eta_dot_0 = eta_dot_0;
    
    
    [F0,Q_a,J] = TIDM_spectral(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config);
    F_ad = F0;
    Q_a = Q_a + Const.Dee*Const.q_dot + Const.Kee*Const.q;

    r = [F0;Q_a] - [F_ad;Q_ad];

    CI = [q_0;r_0;eta_0];
    [~, X3] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X_meas,CI,Config.option); % Solve ODE
    Xt = X3(:,5:7);

    position_disks_simu(:, :, it_t) = Xt;

    if Config.plot
        
        % Tracé de la poutre dans l'espace 3D
    
        figure(1)
        plot(Xt(:,1),Xt(:,3),'b','LineWidth',2)
        grid on
        axis equal
        xlim([-Config.L Config.L])
        ylim([-Config.L Config.L])
        drawnow
    end
    %--------------------------------------------%
%affectation si r<r_min
    q_np1_kp1 = q_np1_k;
    q_dot_np1_kp1 = q_dot_np1_k;
    q_dot_dot_np1_kp1 = q_dot_dot_np1_k;

    %     norm(r(7:end));
%     k = 1;
%     figure(12)
%     plot(k,norm(r(7:end)),'r*')
%     hold on
%     grid on
    
    
    
    while norm(r(7:end)) > r_min        
                
        Delta_q_k =  (-J(7:end,7:end) - a*Const.Dee - Const.Kee)\r(7:end); 
        %
%         avec la partie locomoteur
%         Delta_q_k =  (-J - a*Const.Dee - Const.Kee)\r(7:end);

% mise à jour
        q_np1_kp1 = q_np1_k + Delta_q_k;
        q_dot_np1_kp1 = q_dot_np1_k + (Gamma/(Beta*h))*Delta_q_k;
        q_dot_dot_np1_kp1 = q_dot_dot_np1_k + (1/(Beta*h^2))*Delta_q_k;

        Const.q         = q_np1_kp1;
        Const.q_dot     = q_dot_np1_kp1;
        Const.q_dot_dot = q_dot_dot_np1_kp1;

        [F0,Q_a,J] = TIDM_spectral(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config);
%         F_ad = F0;
        Q_a = Q_a + Const.Dee*Const.q_dot + Const.Kee*Const.q;

        r = [F0;Q_a] - [F_ad;Q_ad];
             
        
        q_np1_k = Const.q;
        q_dot_np1_k = Const.q_dot;
        q_dot_dot_np1_k = Const.q_dot_dot;
        
%         norm(r(7:end));
%     k = k+1;
%     figure(12)
%     plot(k,norm(r(7:end)),'r*')
%     drawnow
    
    end
    
    q(:,it_t+1)         = q_np1_kp1;
    q_dot(:,it_t+1)     = q_dot_np1_kp1;
    q_dot_dot(:,it_t+1) = q_dot_dot_np1_kp1;  
end

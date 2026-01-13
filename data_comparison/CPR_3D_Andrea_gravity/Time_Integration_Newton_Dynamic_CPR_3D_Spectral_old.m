function [T,q] = Time_Integration_Newton_Dynamic_CPR_3D_Spectral_old(t,Const,Config,Var)
% v = VideoWriter('Beam_Till_shooting_act_cable_static.avi');
% open(v);
% Critère résidu Newton
r_min = 1e-6;

% Gain intégrateur de Newmark
Beta  = 1/6; % interpolation 1/4; % valeur moyenne 1/6
Gamma = 1/2;
h = Config.dt;
a = Gamma/(Beta*h);
b = 1/(Beta*h^2);

% Initialisation des sorties 
q(:,1)         = Const.q;
q_dot(:,1)     = Const.q_dot;
q_dot_dot(:,1) = Const.q_dot_dot;
lambda(:,1) = Const.lambda;
T(1) = 0;

%Initialisation de la pose de la tête
q_0   = Const.q_0;
r_0   = Const.r_0;
eta_0 = Const.eta_0;
eta_dot_0 = Const.eta_dot_0;
% Définition du nombre de pas statique
% Pas d'incrément de la force entre chaque calcul de Newton en satique
Pas_F = 0.1;
% Recherche de la valeur max des forces appliquées sur la poutre
Fmax = max(abs([Const.Fm_materielle+Const.Fm_spaciale;
                Const.Fp_materielle+Const.Fp_spaciale;
                Const.Fbar_materielle+Const.Fbar_spaciale]));
% Calcul du nombre de pas statique
N_pas_static = max(ceil(Fmax/Pas_F)-1,0); % -1 pour pas faire le test F =0

% Calcul du nombre de pas temps
N_pas_temps = max(size(t))-1; % Car Newton calcul le t+1 

% Nombre de pas de simulation
N_simu = N_pas_static + N_pas_temps;

% mémoire des forces max
Fm_materielle   = Const.Fm_materielle;
Fm_spaciale     = Const.Fm_spaciale;
Fp_materielle   = Const.Fp_materielle;
Fp_spaciale     = Const.Fp_spaciale;
Fbar_materielle = Const.Fbar_materielle;
Fbar_spaciale   = Const.Fbar_spaciale;

for it_simu = 1:N_simu
    
    if it_simu<=N_pas_static
        iter_F = it_simu/N_pas_static;
        Const.Fm_materielle   = iter_F*Fm_materielle;
        Const.Fm_spaciale     = iter_F*Fm_spaciale;
        Const.Fp_materielle   = iter_F*Fp_materielle;
        Const.Fp_spaciale     = iter_F*Fp_spaciale;
        Const.Fbar_materielle = iter_F*Fbar_materielle;
        Const.Fbar_spaciale   = iter_F*Fbar_spaciale;
        time = 0
        % Calcul du couple généralisé de commande
        Q_ad = zeros(Const.dim_base,1);
%         titre = 'Mise en charge statique';
    else
        time = t(it_simu-N_pas_static+1)
       
        % Calcul du couple généralisé de commande
        L = [];
        for it_troncon = 1:Config.nb_poutre
            Const.it_troncon = it_troncon;

            L_i = L_q_Spectral(Const,Config);
            
            L = [L;L_i];
        end 
        
        % Tensions appliquées aux câbles 
        T_act = 0*time*[75;0 ;0; 
                    0; 20; 10; 
                    20; 30; 40];
        T_act = 0*time*[75;0 ;0];
 
                
         Q_ad = L*T_act;
                
%         titre = ['Traction par les cables T = f(t) et t = ' num2str(time) 's'];
    end

    % Affectation de l'état courant pour la boucle de Newton à t+1 (np1 => n+1)
    q_n = q(:,it_simu);
    q_dot_n = 0*q_dot(:,it_simu);
    q_dot_dot_n = 0*q_dot_dot(:,it_simu);
    lambda_n = lambda(:,it_simu);

    % Prédiction de l'état k=0 dans ce cas
    q_np1_k = q_n + h*q_dot_n +(1/2 - Beta)*h^2*q_dot_dot_n;    
    q_dot_np1_k = q_dot_n +(1 - Gamma)*h*q_dot_dot_n;  
    q_dot_dot_np1_k = 0*q_dot_dot_n;
    
    
    lambda_np1_k = lambda_n;
 
    % Affectation des variable q_np1_k dans la structure pour le calcul 
    Const.q         = q_np1_k;
    Const.q_dot     = q_dot_np1_k;
    Const.q_dot_dot = q_dot_dot_np1_k;
    Const.lambda  = lambda_np1_k; 
    
    % Calcul de la sortie de l'ISM et de la jacobienne à t+1 rt k=0
    [Q_a,J] = TIDM_spectral_SO3(q_0,r_0,eta_0,eta_dot_0,it_simu,Const,Config);
    Q_a = Q_a + [Const.Kee*Const.q+mu*Const.Kee*Const.q_dot; zeros(6,1)];
    
    % Calcul du résidu
    r = Q_a - [Q_ad; zeros(6,1)];

          %calcul num de J
% 
%         for it_D_q = 1:Const.dim_base + 6
%             Delta_q_T = zeros(Const.dim_base,1);
%             Delta_lamba = zeros(6,1);
%             if it_D_q <= Const.dim_base
%                 Delta_q_T(it_D_q,1) = 1e-3;
%             else
%                 Delta_lamba(it_D_q-Const.dim_base,1) = 1e-3;
%             end
%             Const.q         = q_np1_k+Delta_q_T;
%             Const.lambda  = lambda_np1_k+Delta_lamba; 
% 
%         % Calcul de la sortie de l'ISM et de la jacobienne à t+1 et k+1
% 
%         [Q_an,~] = TISM_spectral(q_0,r_0,a,b,time,Const,Config);
%         Q_an = Q_an + [Const.Kee*Const.q; zeros(6,1)];
%         Jn(:,it_D_q) =[Q_an-Q_a]/1e-3;
%         end
% 
%         J2=(-J - [Const.Kee, zeros(Const.dim_base,6); zeros(6,Const.dim_base), zeros(6,6)]);
    Const.q  = q_np1_k;
    Const.q_dot  = q_dot_np1_k;
    Const.q_dot_dot  = q_dot_dot_np1_k;
    Const.lambda  = lambda_np1_k;
    %-------------- trace -----------------%
    [QX,rX] = Reconstruction(q_0,r_0,Const,Config);
    
    % Tracé de la poutre dans l'espace 3D

    fig_1=figure(1);
%     hold off
    for it_trace = 1:Config.nb_poutre
        a_trace = (it_trace-1)*Config.N_noeuds+1;
        b_trace = it_trace*Config.N_noeuds;
%         plot(rX(1,a_trace:b_trace),rX(3,a_trace:b_trace),'r','LineWidth',2)
% hold on
        plot3(rX(1,a_trace:b_trace),rX(2,a_trace:b_trace),rX(3,a_trace:b_trace),'r','LineWidth',2)
%         hold on
%         plot3(Config.L*ones(size(rX(1,a_trace:b_trace))),rX(2,a_trace:b_trace),rX(3,a_trace:b_trace),'LineWidth',2,'Color',[0.8,0.8,0.8])
%         plot3(rX(1,a_trace:b_trace),Config.L*ones(size(rX(2,a_trace:b_trace))),rX(3,a_trace:b_trace),'LineWidth',2,'Color',[0.8,0.8,0.8])
%         plot3(rX(1,a_trace:b_trace),rX(2,a_trace:b_trace),0*ones(size(rX(3,a_trace:b_trace))),'LineWidth',2,'Color',[0.8,0.8,0.8])

    end
    grid on
    axis equal
    xlim([-0.6*Config.L Config.L])
    ylim([-0.35*Config.L 0.35*Config.L])
    zlim([-0.35*Config.L 0.35*Config.L])
%     titre = ['\theta_d = ' num2str(Config.Theta_d(1,time)*180/pi) ', r_d = ' num2str(Config.r_d(1,time))];
%     title(titre);

    drawnow
    frame = getframe(fig_1);
    writeVideo(v,frame);

    %----------------- fin trace -----------------------%
    
    % Affectation de l'état courant si r < r_min 
    q_np1_kp1 = q_np1_k;
    q_dot_np1_kp1 = q_dot_np1_k;
    q_dot_dot_np1_kp1 = q_dot_dot_np1_k;
    
    lambda_np1_kp1 = lambda_np1_k;
    Delta_q_k = ones(Const.dim_base+6,1);
%     k=1;
%     figure(10)
%     plot(k,norm(r),'r*')
%     hold on 
%     grid on
    while norm(r) > r_min% && norm(Delta_q_k)>1e-8
        % Calcul de la variation Delta_q
%         Delta_q_k =  (-J - [Const.Kee, zeros(Const.dim_base,6); zeros(6,Const.dim_base), zeros(6,6)])\r;
        Delta_q_k =  pinv(-J - [Const.Kee, zeros(Const.dim_base,6); zeros(6,Const.dim_base), zeros(6,6)])*r;
%         Delta_q_k =  inv(-Jn)*r;

        % Mise à jour de l'état
        q_np1_kp1 = q_np1_k + Delta_q_k(1:Const.dim_base,1);
        q_dot_np1_kp1 = q_dot_np1_k + (Gamma/(Beta*h))*Delta_q_k(1:Const.dim_base,1);
        q_dot_dot_np1_kp1 = q_dot_dot_np1_k + (1/(Beta*h^2))*Delta_q_k(1:Const.dim_base,1);
        lambda_np1_kp1 = lambda_np1_k + Delta_q_k(Const.dim_base+1:end,1);


        % affectation des variable q_np1_k dans la structure pour le calcul 
        Const.q         = q_np1_kp1;
        Const.q_dot     = q_dot_np1_kp1;
        Const.q_dot_dot = q_dot_dot_np1_kp1;
        Const.lambda  = lambda_np1_kp1; 

        % Calcul de la sortie de l'ISM et de la jacobienne à t+1 et k+1

        [Q_a,J] = TIDM_spectral_SO3(q_0,r_0,eta_0,eta_dot_0,it_simu,Const,Config);
        Q_a = Q_a + [Const.Kee*Const.q; zeros(6,1)];

        % Calcul du résidu
        r = Q_a - [Q_ad; zeros(6,1)];
             
        % Affectation de l'état courant k+1 -> k (changement d'indice de la boucle de Newton) 

        q_np1_k = Const.q;
        q_dot_np1_k = Const.q_dot;
        q_dot_dot_np1_k = Const.q_dot_dot;
        lambda_np1_k = Const.lambda; 
    
        
                  %calcul num de J
% 
%         for it_D_q = 1:Const.dim_base + 6
%             Delta_q_T = zeros(Const.dim_base,1);
%             Delta_lamba = zeros(6,1);
%             if it_D_q <= Const.dim_base
%                 Delta_q_T(it_D_q,1) = 1e-3;
%             else
%                 Delta_lamba(it_D_q-Const.dim_base,1) = 1e-3;
%             end
%             Const.q         = q_np1_k+Delta_q_T;
%             Const.lambda  = lambda_np1_k+Delta_lamba; 
% 
%         % Calcul de la sortie de l'ISM et de la jacobienne à t+1 et k+1
% 
%         [Q_an,~] = TISM_spectral(q_0,r_0,a,b,time,Const,Config);
%         Q_an = Q_an + [Const.Kee*Const.q; zeros(6,1)];
%         Jn(:,it_D_q) =[Q_an-Q_a]/1e-3;
%         end

%       k=k+1;
%     figure(10)
%     plot(k,norm(r),'r*')
%     hold on 
%     grid on      
%         [norm(r),norm(Delta_q_k)]
    
    end
    
    % Affectation de l'état convergé r<r_min aux sortie puis itération du
    % temps
    q(:,it_simu+1)         = q_np1_kp1;
    q_dot(:,it_simu+1)     = q_dot_np1_kp1;
    q_dot_dot(:,it_simu+1) = q_dot_dot_np1_kp1;
    lambda(:,it_simu+1)    = lambda_np1_kp1;
    T(it_simu+1) = time;

end

close(v);
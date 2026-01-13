function   dy = Calcul_q_dot(X,y,t,Const,Config)

% calcul la première ode spaciale 
% Integration spaciale de 0->1
% entrées

q   = y; 

% Position et orientation initiales de la tete

r_0 = [0;0;0];
Q_0 = [1 0 0 0]';
Q_0 = [0.7071068 0 0.7071068 0]';

eta_0 = [0 0 0 0 0 0]'; % vitesse de la tête
time = 0;

% Calcul des efforts de gravité (F et Q) pour g0 et r cst 
% et r_dot = 0, r_dot_dot =0, eta0 = 0 et eta0_dot=0. 

Const.q         = q;

Const.q_dot     = zeros(Const.dim_base,1);
Const.q_dot_dot = zeros(Const.dim_base,1);
Const.eta_dot   = zeros(6,1);

[F0,Q_a] = IDM_ORIN(Q_0,r_0,eta_0,time,Const,Config);

Fc = F0;
Qc = Q_a';

% Calcul de q_dot
q_dot = inv(Const.Dee)*(-Const.Kee*q-Qc);

% Visualisation du transitoire

% figure(1)
% plot(X,q,'.')
% grid on
% hold on
% drawnow
% % 
% figure(2)
% plot(X,q_dot,'.')
% grid on
% hold on
% drawnow

% Reconstruction de la poutre pour visualisation du transitoire

if Config.Trace_transitoire==1
    
    CI = [Q_0;r_0;eta_0];

    [X X3] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X,CI,Config.option); % Solve ODE

    % Affectation des valeurs pour le tracé

    Xt = X3(:,5:7);

    % Tracé de la poutre dans l'espace 3D

    figure(4)
    plot(Xt(:,1),Xt(:,3),'LineWidth',2)
    grid on
    axis equal
    xlim([-Config.L Config.L])
    ylim([-Config.L Config.L])
    drawnow
end

% sorties
dy = q_dot;

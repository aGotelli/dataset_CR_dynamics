function [q,q_dot,q_dot_dot,position_disks_simu,wrench_at_base,cables_displacements] = forward_dynamics_simulation(Const,Config)
% forward_dynamics_simulation  Newmark-beta time integration of the GVS
%   rod's modal coordinates q, driven by the measured tendon tensions in
%   Config.data.tau, with a Newton-Raphson correction at each step using
%   TIDM's generalised force and Jacobian.

r_min = 1e-6;
Beta  = 1/4; % average-acceleration Newmark scheme
Gamma = 1/2;

h = Config.data.dt;
a = Gamma/(Beta*h);
b = 1/(Beta*h^2);

q(:,1)         = Const.q;
q_dot(:,1)     = Const.q_dot;
q_dot_dot(:,1) = Const.q_dot_dot;

q_0       = Const.q_0;
r_0       = Const.r_0;
eta_0     = Const.eta_0;
eta_dot_0 = Const.eta_dot_0;

qn_0 = Const.q_0;
rn_0 = Const.r_0;

data = Config.data;
time = data.time;

N_time = length(time);
position_disks_simu  = zeros(5,3,N_time);
wrench_at_base        = zeros(N_time,6);
cables_displacements  = zeros(N_time,2);

for it_t = 1:N_time

    tau = data.tau(:,it_t);

    q_n         = q(:,it_t);
    q_dot_n     = q_dot(:,it_t);
    q_dot_dot_n = q_dot_dot(:,it_t);

    % Newmark prediction
    q_np1_k         = q_n + h*q_dot_n + (1/2-Beta)*h^2*q_dot_dot_n;
    q_dot_np1_k     =         q_dot_n + (1-Gamma)*h*q_dot_dot_n;
    q_dot_dot_np1_k = 0*q_dot_dot_n;

    Const.q         = q_np1_k;
    Const.q_dot     = q_dot_np1_k;
    Const.q_dot_dot = q_dot_dot_np1_k;

    %   Actuation
    Q_ad = internalActuation(tau,Const,Config);

    [~,Q_a,J] = TIDM(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config);
    Residual = Q_a + Const.Dee*Const.q_dot + Const.Kee*Const.q - Q_ad;
    Jacobian = J(7:end,7:end) + a*Const.Dee + Const.Kee;

    iter = 0;
    while norm(Residual) > r_min

        Delta_q_k = -Jacobian\Residual;

        q_np1_kp1         = q_np1_k + Delta_q_k;
        q_dot_np1_kp1     = q_dot_np1_k + (Gamma/(Beta*h))*Delta_q_k;
        q_dot_dot_np1_kp1 = q_dot_dot_np1_k + (1/(Beta*h^2))*Delta_q_k;

        Const.q         = q_np1_kp1;
        Const.q_dot     = q_dot_np1_kp1;
        Const.q_dot_dot = q_dot_dot_np1_kp1;

        Q_ad = internalActuation(tau,Const,Config);
        [~,Q_a,J] = TIDM(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config);
        Residual = Q_a + Const.Dee*Const.q_dot + Const.Kee*Const.q - Q_ad;
        Jacobian = J(7:end,7:end) + a*Const.Dee + Const.Kee;

        q_np1_k         = Const.q;
        q_dot_np1_k     = Const.q_dot;
        q_dot_dot_np1_k = Const.q_dot_dot;

        iter = iter+1;
    end

    q(:,it_t+1)         = Const.q;
    q_dot(:,it_t+1)     = Const.q_dot;
    q_dot_dot(:,it_t+1) = Const.q_dot_dot;

    %   Base reaction wrench
    [F0,~,~] = TIDM(qn_0,rn_0,q_0,r_0,eta_0,eta_dot_0,a,b,time,Const,Config);
    wrench_at_base(it_t,:) = F0';

    %   Disk positions (for comparison against mocap)
    CI = [q_0;r_0;eta_0];
    [~,X3] = ode45(@(t,y) Forward_eta(t,y,time,Const,Config),Config.X_meas,CI,Config.option);
    Xt = X3(:,5:7);

    position_disks_simu(:,:,it_t) = Xt;

    %   Cable displacements
    [pulled_lengths,~] = getCablesLength(Const,Config);
    cables_displacements(it_t,:) = pulled_lengths';

    if Config.plot
        % Rod shape in 3D
        figure(1)
        plot3(Xt(:,1),Xt(:,2),Xt(:,3),'b','LineWidth',2)
        grid on
        axis equal
        xlim([-Config.L Config.L])
        ylim([-Config.L Config.L])
        zlim([-Config.L Config.L])
        drawnow
    end

    fprintf('\n%-12s %-12s %-12s %-12s %-12s\n','time','tau_1','tau_2','NR iter','% complete');
    fprintf('%-12.4f %-12.4f %-12.4f %-12d %-12.2f\n',time(it_t),tau(1),tau(2),iter,(it_t/N_time)*100);
end

end

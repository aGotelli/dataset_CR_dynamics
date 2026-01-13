function J = getJacobian(t, q, dot_q, ddot_q, Const, Config)



n = Const.dim_base;

a = Config.a;
b = Config.b;


Delta_q_stack      = eye(n, n);
Delta_dot_q_stack  = a*eye(n, n);
Delta_ddot_q_stack = b*eye(n, n);


J = zeros(n, n);



for it=1:n
      
    %   Get current set of variations
    Delta_q      = Delta_q_stack(:, it);
    Delta_dot_q  = Delta_dot_q_stack(:, it);
    Delta_ddot_q = Delta_ddot_q_stack(:, it);



    [~, Delta_Qa_X0] = TIDM(t, q, dot_q, ddot_q, ...
                            Delta_q, Delta_dot_q, Delta_ddot_q, ...
                            Config, Const);

    %   Compose Jacobian column
    J(:, it) = Const.Kee*Delta_q + Const.Dee*Delta_dot_q - Delta_Qa_X0;
end




end
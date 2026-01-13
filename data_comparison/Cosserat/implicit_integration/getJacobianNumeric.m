function J = getJacobianNumeric(t, q, dot_q, ddot_q, Const, Config)



n = Const.dim_base;

a = Config.a;
b = Config.b;

delta = Config.delta;


Delta_q_stack      = eye(n, n);
Delta_dot_q_stack  = a*eye(n, n);
Delta_ddot_q_stack = b*eye(n, n);


J = zeros(n, n);


R = getResidual(t, q, dot_q, ddot_q, Const, Config);



for it=1:n
      
    %   Get current set of variations
    delta_q      = q;
    delta_dot_q  = dot_q;
    delta_ddot_q = ddot_q;

    delta_q(it)      = q(it)      + delta;
    delta_dot_q(it)  = dot_q(it)  + a*delta;
    delta_ddot_q(it) = ddot_q(it) + b*delta;


    delta_R = getResidual(t, delta_q, delta_dot_q, delta_ddot_q, Const, Config);


    %   Compose Jacobian column
    J(:, it) = (delta_R - R)/delta;
end




end
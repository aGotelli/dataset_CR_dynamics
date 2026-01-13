
function J = getJacobianNumericWithStateEstimation(t, q, dot_q, ddot_q, Const, Config, Measures)



n = Const.dim_base;

a = Config.a;
b = Config.b;

delta = Config.delta;


[~, R] = getResidualWithStateEstimation(t, q, dot_q, ddot_q, Const, Config, Measures);

Jq   = zeros(n, n);
Jdq  = zeros(n, n);
Jddq = zeros(n, n);


for it=1:n
      
    %   Get current set of variations
    delta_q      = q;
    delta_q(it)      = q(it)      + delta;

    %   Compute influence on residual
    [~, delta_R] = getResidualWithStateEstimation(t, delta_q, dot_q, ddot_q, Const, Config, Measures);

    %   Compose Jacobian column
    Jq(:, it) = (delta_R - R)/delta;
end


for it=1:n
      
    %   Get current set of variations
    delta_dot_q  = dot_q;
    delta_dot_q(it)  = dot_q(it)  + delta;

    %   Compute influence on residual
    [~, delta_R] = getResidualWithStateEstimation(t, q, delta_dot_q, ddot_q, Const, Config, Measures);

    %   Compose Jacobian column
    Jdq(:, it) = (delta_R - R)/delta;
end



for it=1:n
      
    %   Get current set of variations
    delta_ddot_q = ddot_q;
    delta_ddot_q(it) = ddot_q(it) + delta;

    %   Compute influence on residual
    [~, delta_R] = getResidualWithStateEstimation(t, q, dot_q, delta_ddot_q, Const, Config, Measures);

    %   Compose Jacobian column
    Jddq(:, it) = (delta_R - R)/delta;
end


J = Jq + a*Jdq + b*Jddq;

end
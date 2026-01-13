function cable_length = getCableLength(q, Const, Config)

%   Initialization
y0 = [Const.Q_X0;
      Const.r_X0;
      0];

%   Integrate the cableTangent function to get the lenght
[~, Y] = ode45(@(X, y) cableTangent(X, y, q, Const), Config.forward_integration_domain, y0);

cable_length = Y(end, end);

end
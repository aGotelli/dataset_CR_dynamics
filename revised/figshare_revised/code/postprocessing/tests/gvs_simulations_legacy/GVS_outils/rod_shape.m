function xyz = rod_shape(t, q, Const, Config)


ne = Const.dim_base;


%   Define the forward state
forward_y0 = [Const.q_0;
                    Const.r_0];

%   Forward integration
[~, Y] = ode45(@(X, y) forward_g(X, y, q, Config, Const), ...
                    Config.observation_points, forward_y0);


xyz = Y(:, 5:7);

end


function dydx = forward_g(X, y, q, Config, Const)

% The state has the form
%     | Q |   w, x, y, z                  1-4
%     | r |   x, y, z                     5-7


%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;


%   Compute strains
Phi = Base_Phi(X,0,Const,Config)';

% Xi      = B*Phi*q + Xi_c;
Xi      = B*Phi*q + Const.B_bar*Const.Xi_c;

K     = Xi(1:3);
Gamma = Xi(4:6);

%  Unpack state vector
Q       = y(1:4);


%   Compute the derivatives
Q_prime       =   1/2*A(K)*Q;
r_prime       =   getR(Q)*Gamma;


%  Packing state vector derivative
dydx = L*[Q_prime;
          r_prime
];
end
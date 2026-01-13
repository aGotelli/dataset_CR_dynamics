function [dydx] = BackwardTangentDynamics(X, y, q, dot_q, ddot_q, Delta_q, Delta_dot_q, Delta_ddot_q, Config, Const)

% The state has the form
%     | Q  |   w, x, y, z                          1 - 4
%     | r  |   x, y, z                             5 - 7
%     | η  |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3         8 - 13
%     | η̇  |   Ω1 , Ω2 , Ω3 , V1 , V2 , V3        14 - 19
%     | ∆ζ |   ∆Π1 , ∆Π2 , ∆Π3 , ∆P1 , ∆P2 , ∆P3  20 - 25
%     | ∆η |   ∆Ω1 , ∆Ω2 , ∆Ω3 , ∆V1 , ∆V2 , ∆V3  26 - 31
%     | ∆η̇ |   ∆Ω1 , ∆Ω2 , ∆Ω3 , ∆V1 , ∆V2 , ∆V3  32 - 37
%     | Λ  |    C1 ,  C2 ,  C3 ,  N1 ,  N2 ,  N3  38 - 43
%     | ∆Λ |   ∆C1 , ∆C2 , ∆C3 , ∆N1 , ∆N2 , ∆N3  44 - 49
%     | ∆Qa|                                      50 - end


%   Obtain the need variables
B     = Const.B;
Xi_c  = Const.Xi_c;
L     = Const.L;
M     = Const.M_cal;

%   Compute strains
Phi = getPhi(X, Const);

Xi      = B*Phi*q + Xi_c;
dot_Xi  = B*Phi*dot_q;
ddot_Xi = B*Phi*ddot_q;

Delta_Xi      = B*Phi*Delta_q;
Delta_dot_Xi  = B*Phi*Delta_dot_q;
Delta_ddot_Xi = B*Phi*Delta_ddot_q;

K     = Xi(1:3);
Gamma = Xi(4:6);

Delta_K = Delta_Xi(1:3);

%  Unpack state vector
Q               = y(1:4);
eta             = y(8:13);
dot_eta         = y(14:19);
Delta_zeta      = y(20:25);
Delta_eta       = y(26:31);
Delta_dot_eta   = y(32:37);
Lambda          = y(38:43);
Delta_Lambda    = y(44:49);

%   Get the external force
F_bar       = F_gravity(Q, Const);
Delta_F_bar = Delta_F_gravity(Q, Delta_K, Const);

%   Compute the derivatives
dydx_tangent_kinematics = ForwardTangentKinematics(X, y, q, dot_q, ddot_q,...
                                                   Delta_q, Delta_dot_q, Delta_ddot_q,...
                                                   Config, Const);
Lambda_prime            = ad_(Xi)'*Lambda + M*dot_eta - ad_(eta)'*M*eta - F_bar;
Delta_Lambda_prime      = M*Delta_dot_eta - ad_(eta)'*M*Delta_eta - ad_(Delta_eta)'*M*eta ...
                            + ad_(Xi)'*Delta_Lambda + ad_(Delta_Xi)'*Lambda - Delta_F_bar;
Delta_Qa_prime          = -Phi'*B'*Delta_Lambda;

%  Packing state vector derivative
dydx = L*[dydx_tangent_kinematics;
          Lambda_prime;
          Delta_Lambda_prime;
          Delta_Qa_prime];


end
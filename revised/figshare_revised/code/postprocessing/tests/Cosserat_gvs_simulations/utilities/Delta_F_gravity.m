function Delta_Fg = Delta_F_gravity(Q, Delta_K, Const)

R = getR(Q);




A   = Const.Area;
g   = Const.g;
rho = Const.rho;

fg = -R'*[0;   
         0; 
         rho*A*g];

Delta_fg = hat(Delta_K)'*fg;

Delta_Fg = [zeros(3,1);
            Delta_fg];




end
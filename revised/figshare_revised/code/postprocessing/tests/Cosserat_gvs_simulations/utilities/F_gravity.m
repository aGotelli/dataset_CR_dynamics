function Fg = F_gravity(Q, Const)

R = getR(Q);


Ng = R'*Const.fg;

Fg = [zeros(3,1);
      Ng];


end
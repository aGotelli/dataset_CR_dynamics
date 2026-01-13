function V = anti_hat(M)

M = (M-M')/2;

V = [-M(2,3);M(1,3);-M(1,2)];

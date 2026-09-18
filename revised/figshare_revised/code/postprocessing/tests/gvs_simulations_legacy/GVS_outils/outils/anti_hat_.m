function V = anti_hat_(M)

M = (M-M')/2;

V = [-M(2,3);M(1,3);-M(1,2)];

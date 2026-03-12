function g_inv = inv_g(g)

R = g(1:3, 1:3);
r = g(1:3, 4);
g_inv = [
    R' -R'*r
    0  0  0  1
];

end
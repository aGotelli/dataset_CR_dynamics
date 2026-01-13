function dr = r_dot(Q,V)

R = quaternion_to_matrice(Q);

dr = R*V;

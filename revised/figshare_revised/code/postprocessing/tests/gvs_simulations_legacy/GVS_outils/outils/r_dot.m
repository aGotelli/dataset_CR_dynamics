function dr = r_dot(Q,V)

Q=Q/norm(Q);
R = quaternion_to_matrice(Q);

dr = R*V;

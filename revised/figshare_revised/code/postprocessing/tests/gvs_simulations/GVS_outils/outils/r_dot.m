function dr = r_dot(Q,V)
% r_dot  Spatial derivative of the position vector r, obtained by
%        rotating the material-frame rate V into the reference frame
%        through the rotation matrix of quaternion Q.

Q = Q/norm(Q);
R = quaternion_to_matrice(Q);

dr = R*V;

end

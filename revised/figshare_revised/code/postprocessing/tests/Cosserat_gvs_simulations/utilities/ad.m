function ad_Xi = ad(Xi)

K = Xi(1:3);
G = Xi(4:6);

ad_Xi = [hat(K), zeros(3, 3);
         hat(G),    hat(K) ];

end
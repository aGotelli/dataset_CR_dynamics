function X = integration_spectral(N_noeuds,X_grille,DX,n,fA,fB,CI,loc_CI)

n = 4;

D = [];   
A = zeros(n*N_noeuds,n*N_noeuds);
b = zeros(n*N_noeuds,1);

for it = 1:n
    D = blkdiag(D,DX);
end

P = eye(size(A));

for i=2:n
    P_sauv=P(:,i);
    P(:,i)=P(:,(i-1)*N_noeuds+1);
    P(:,(i-1)*N_noeuds+1)=P_sauv;
end


D = P'*D*P;
b = P'*b;

for it_x = 1:N_noeuds

    Omega =  Xi(1:3,it_x);

    A_Omega = [       0, -Omega(1), -Omega(2), -Omega(3)
              Omega(1),         0,  Omega(3), -Omega(2)
              Omega(2), -Omega(3),         0,  Omega(1)
              Omega(3),  Omega(2), -Omega(1),        0];


    a_Q  = 1/2*A_Omega;
    A_Q11(it_x,it_x) = a_Q(1,1);
    A_Q12(it_x,it_x) = a_Q(1,2);
    A_Q13(it_x,it_x) = a_Q(1,3);
    A_Q14(it_x,it_x) = a_Q(1,4);
    A_Q21(it_x,it_x) = a_Q(2,1);
    A_Q22(it_x,it_x) = a_Q(2,2);
    A_Q23(it_x,it_x) = a_Q(2,3);
    A_Q24(it_x,it_x) = a_Q(2,4);
    A_Q31(it_x,it_x) = a_Q(3,1);
    A_Q32(it_x,it_x) = a_Q(3,2);
    A_Q33(it_x,it_x) = a_Q(3,3);
    A_Q34(it_x,it_x) = a_Q(3,4);
    A_Q41(it_x,it_x) = a_Q(4,1);
    A_Q42(it_x,it_x) = a_Q(4,2);
    A_Q43(it_x,it_x) = a_Q(4,3);
    A_Q44(it_x,it_x) = a_Q(4,4);
end

A_Q = [A_Q11,A_Q12,A_Q13,A_Q14;A_Q21,A_Q22,A_Q23,A_Q24;A_Q31,A_Q32,A_Q33,A_Q34;A_Q41,A_Q42,A_Q43,A_Q44];           
A_Q=P_Q'*A_Q*P_Q;
CL=(D_Q(:,1:n_Q)-A_Q(:,1:n_Q))*q_0;
res=(D_Q(n_Q+1:end,n_Q+1:end)-A_Q(n_Q+1:end,n_Q+1:end))\(b_Q(n_Q+1:end,1)-CL(n_Q+1:end,1));

Q_x=P_Q*[q_0;res];

Q_x_M = reshape(Q_x,[N_noeuds,n_Q]);   

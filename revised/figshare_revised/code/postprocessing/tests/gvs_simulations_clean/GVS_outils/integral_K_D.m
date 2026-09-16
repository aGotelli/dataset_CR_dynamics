function dy = integral_K_D(X,~,Const,Config)
% integral_K_D  Integrand of computeGeneralisedStiffnessDampingMatrices:
%               the modal projections of the cross-sectional stiffness
%               (Ha) and damping (Da) matrices at X.

B = Const.B;

H = Const.H_cal;
D = Const.mu*H;

Ha = B'*H*B;
Da = B'*D*B;

Phi = Base_Phi(X,0,Const,Config)';

Krr_prime = Phi'*Ha*Phi;
Drr_prime = Phi'*Da*Phi;

dy = [
    reshape(Krr_prime,[Const.dim_base*Const.dim_base,1]);
    reshape(Drr_prime,[Const.dim_base*Const.dim_base,1])
];

end

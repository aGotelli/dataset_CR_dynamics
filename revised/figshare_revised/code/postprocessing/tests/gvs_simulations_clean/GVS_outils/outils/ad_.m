function M = ad_(G)
% ad_  se(3) adjoint matrix of a twist G = [omega;v] (6x1), used to
%      express the Lie bracket ad_(G)*H = [G,H] as a matrix product.

W = G(1:3);
U = G(4:6);

ad11 = hat_(W);
ad12 = zeros(3);
ad21 = hat_(U);

M = [[ad11,ad12];[ad21,ad11]];

end

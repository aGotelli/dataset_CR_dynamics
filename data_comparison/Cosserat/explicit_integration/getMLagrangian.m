function M = getMLagrangian(t, y, Const, Config)


%   Get dimension of the base
ne = Const.dim_base;

q     = y(1:ne);
zeroq = zeros(ne, 1);

%   Compute with only q
[~, Qa_X0] = IDM(t, q, zeroq, zeroq, Config, Const);

%   Get configuration dependent efforts
Qc = Qa_X0;



%   Define all the deltas at once
delta_matrix = eye(Const.dim_base, Const.dim_base);

%   Pepare matrix for M
M = zeros(Const.dim_base, Const.dim_base);

%   Loop through all the coordinates to fill M
for it=1:ne
    
    %   Extract delta
    delta = delta_matrix(:, it);

    %   IDM with unitary entry for ddot_q
    [~, Qa_X0] = IDM(t, q, zeroq, delta, Config, Const);


    %   Fill the matrix
    M(:, it) = Qa_X0 - Qc;


end





end
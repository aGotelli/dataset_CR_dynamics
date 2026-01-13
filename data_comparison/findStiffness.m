function error_norm = findStiffness(EI, q, Const, Config)
    Const.EI = EI;

    positions_simu = find_rod_state_NR(q, Const, Config);


    %   Compare with data
    error = positions_simu - Const.position_data;

    [index, count] = find(Const.D ~= 0);
    P = zeros(3, 3);
    P(1, 1) = 1;
    for it=1:count
        n = index(it);
        P(n, n) = 1;
    end
    
    actual_error = P*error;
    
    %   Scalar varible -> scalar output
    error_norm = norm(actual_error);
    
end
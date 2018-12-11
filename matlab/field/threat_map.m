function res = threat_map(x, y, px, py, vx, vy)
    sigma1 = 0.1;
    sigma2 = 0.01;
    omega = [sigma1 0; 0 sigma2]
    alpha = 0.2;
    
    pos_err = [x-px ; y-py]
    v_vec = [vx; vy];
    
    res = exp(-pos_err' * omega * pos_err) / (1 + exp(-alpha*v_vec'*pos_err));
    %res = 1.0/(1 + exp(-alpha*v_vec'*pos_err));
end
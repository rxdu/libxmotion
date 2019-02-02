function res = threat_map(x, y, px, py, vx, vy)

    sigma_x = 0.03;
    sigma_y = 0.15;
%    omega = [sigma_x 0; 0 sigma_y];
%    alpha = 0.03;
    theta = atan2(vy, vx);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    omega = R*[sigma_x 0; 0 sigma_y]*R';
    alpha = 0.05;
    
    pos_err = [x-px ; y-py]
    v_vec = [vx; vy];
    
    % a different way of getting bias
    %res = exp(-pos_err' * omega * pos_err) * (1 + 2.0/pi * atan(alpha*v_vec'*pos_err));
    
    % according to the MIT paper
    res = exp(-pos_err' * omega * pos_err) / (1 + exp(-alpha*v_vec'*pos_err));
    
    % other experiments
    %res = exp(-pos_err' * omega * pos_err) / (1 + exp(-alpha*v_vec'*pos_err));
    %res = log(exp(-pos_err' * omega * pos_err) / (1 + exp(-alpha*v_vec'*pos_err)));
    %res = 1.0/(1 + exp(-alpha*v_vec'*pos_err));
    
end
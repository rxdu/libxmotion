clear;
clc;

r = 4;
n = 2 * r - 1;
dim = 1;
t0 = 0;
t1 = 1;
d = 1;

% m = 1;
% tDes = [0;1.2];
% posDes(:, :, 1) = [-0.15 0.25; 0.0 0.0; 0 0; 0 0; 0 0; 0 0];
% 
% Q = get_hessian(n, r, t0, t1); 
% Q = 1./((tDes(2, 1)-tDes(1, 1))^(2*r-1)).*Q
% [A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)
%
% syms sig0 sig1 sig2 sig3
% x = [ sig0, sig1, sig2, sig3];
% x * Q * transpose(x)
% 
% A_eq * transpose(x)
% b_eq

m = 2; %number of pieces in trajectory
tDes = [0;1.2; 3];
posDes(:, :, 1) = [-0.15 0.25 0.3; 0 0 0; 0 0 0; 0 0 0; 0 0 0];

% Q_joint = [];
% for i = 1:m,
%     Q = get_hessian(n, r, t0, t1); % find cost matrix for each segment
%     Q = 1./((tDes(i+1, 1)-tDes(i, 1))^(2*r-1)).*Q;% multiply by time factor to nondimensionalize
%     
%     Q_joint = blkdiag(Q_joint, Q);%put in block diagonal matrix
% end
% Q_joint;
% [A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes);
% 
% syms sig0 sig1 sig2 sig3
% x = [ sig0, sig1, sig2, sig3];
% x * Q_joint * transpose(x)
% 
% A_eq * transpose(x)
% b_eq

for i = 1:d
    xT(:, :, i) = find_polynomial(r, n, m, i, tDes, posDes)
end


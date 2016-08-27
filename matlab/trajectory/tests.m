clear;
clc;

r = 2;
n = 2 * r - 1;
dim = 1;
t0 = 0;
t1 = 1;

m = 1;
tDes = [0;1.2];
posDes(:, :, 1) = [-0.15 0.25; 0.0 0.0; 0 0; 0 0; 0 0; 0 0];


Q = get_hessian(n, r, t0, t1); 
Q = 1./((tDes(2, 1)-tDes(1, 1))^(2*r-1)).*Q
[A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)


% m = 2; %number of pieces in trajectory
% tDes = [0;1.2; 3];
% posDes(:, :, 1) = [-0.15 0.25 0.3; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0];
% [A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)

% syms sig0 sig1 sig2 sig3
% x = [ sig0, sig1, sig2, sig3];
% x * Q * transpose(x)
% 
% A_eq * transpose(x)
% b_eq

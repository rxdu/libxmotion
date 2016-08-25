clear;
clc;

r = 4;
n = 2 * r - 1;
m = 1;
dim = 1;
tDes = [0;1.2];
posDes(:, :, 1) = [-0.15 0.25; 0.1 0.2; 0 0; 0 0; 0 0; 0 0];
t0 = 0;
t1 = 1;
[A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)

% r = 2; %derivative to minimize in cost function
% n = 3; %order of desired trajectory
% m = 2; %number of pieces in trajectory
% d = 1; %dimensions
% dim = 1;
% 
% tDes = [0;1.2; 3];
% posDes(:, :, 1) = [-0.15 0.25 0.3; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0; 0 Inf 0];
% t0 = 0;
% t1 = 1;
% [A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)

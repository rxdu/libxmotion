clear;
clc;

r = 2;
n = 2 * r - 1;
m = 1;
dim = 1;
tDes = [0;1.2];
posDes(:, :, 1) = [-0.15 0.25; 0.1 0.2; 0 0; 0 0; 0 0; 0 0];
t0 = 0;
t1 = 1;
[A_eq, b_eq] = get_all_constraints(r, n, m, dim, posDes, t0, t1, tDes)

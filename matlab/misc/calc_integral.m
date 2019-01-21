close all;clear;clc;

syms x1 x2 xn1 xn2 
syms p1 p2 v1 v2
syms sigma alpha
syms omega11 omega12 omega21 omega22
syms c

x = [x1; x2];
p = [p1; p2];
v = [v1; v2];
xn = [xn1; xn2];
Omega = [omega11 omega12; omega21 omega22];

func = exp(-(x-p)'*Omega*(x-p) - 1./(4*sigma^2)*(x-xn)'*(x-xn))/(1 + exp(-alpha* v'*(x-p)));

% res = simplify(int(int(func, x1), x2))
% pretty(res)
solve(func == c, x1)

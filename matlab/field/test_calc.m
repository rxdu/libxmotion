% main script
close all; clear; clc;

syms x x0 y y0 real
syms sigx sigy theta real

Err = [x; y] - [x0; y0]

R = [cos(theta) -sin(theta); sin(theta) cos(theta)]
Sig = [1/(2*sigx^2) 0; 0 1/(2*sigy^2)]

Omega = R * Sig * R';

disp('result:')

pretty(simplify(Omega))

% Res = -Err' * Omega * Err;
% 
% pretty(simplify(Res))

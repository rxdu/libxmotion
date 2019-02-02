% main script
close all; clear; clc;

syms x x0 y y0 real
syms sigf sigs beta real

Err = [x; y] - [x0; y0]

R = [cos(beta) -sin(beta); sin(beta) cos(beta)]
Sig = [1/(2*sigf^2) 0; 0 1/(2*sigs^2)]

Omega = R * Sig * R';

disp('result:')

pretty(simplify(Omega))

% Res = -Err' * Omega * Err;
% 
% pretty(simplify(Res))

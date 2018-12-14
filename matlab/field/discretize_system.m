% main script
close all; clear; clc;

% A = [0 1;
%     0 0]
% B = [0;
%     1]

A = [0 0 1 0;
    0 0 0 1;
    0 0 0 0;
    0 0 0 0]
B = [0 0;
    0 0;
    1 0;
    0 1]

syms dt

Ad = eye(size(A,1)) + dt * A
Bd = (dt * eye(size(A,1)) + 1/2 * dt^2 * A) * B

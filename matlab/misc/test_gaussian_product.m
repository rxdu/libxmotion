close all;clear;clc;

format long
%syms x1 x2
%x = [x1; x2]

sigma = 1;
x1hat = 0;
x2hat = 0;

% coeff1 = 1/ (2 * pi * sigma^2);
% coeff2 = -1/(2*sigma^2);
% coeff1 = 1/ (4 * pi^2 * sigma^4);
% coeff2 = -1/(sigma^2);
% 
% pn1 = @(x1,x2) coeff1 * exp(coeff2 *([x1; x2] - [x1hat; x2hat])'* ([x1; x2] - [x1hat; x2hat]));
% pn2 = @(x1,x2) coeff1 .* exp(coeff2 .* ((x1 - x1hat).^2 +(x2-x2hat).^2));
% 
% pn1(0,0)
% pn2(0,0)
% integral2(pn2, -50, 50, -50, 50)

coeff1 = 1/ (sqrt(2 * pi) * sigma);
coeff2 = -1/(4 * sigma^2);

pn3 = @(x1,x2) (coeff1 .* exp(coeff2 .* ((x1 - x1hat).^2 +(x2-x2hat).^2))) .* (coeff1 .* exp(coeff2 .* ((x1 - x1hat).^2 +(x2-x2hat).^2)));
integral2(pn3, -50, 50, -50, 50)

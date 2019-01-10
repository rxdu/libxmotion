close all;clear;clc;

[x,y] = meshgrid(-10:0.1:10,-10:0.1:10);

sigma = 1;
x1hat = 0;
x2hat = 0;

coeff1 = 1/ (sqrt(2 * pi) * sigma);
coeff2 = -1/(4 * sigma^2);

z = coeff1 .* exp(coeff2 .* ((x - x1hat).^2 +(y - x2hat).^2));

surf(x,y,z)
pbaspect([1 1 1])

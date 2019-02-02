close all; clear; clc;

Pi = [0,0];
step = 0.1;
size_x = 1.8;
size_y = 3;
[fx,fy] = meshgrid(-15:step:15);

%% position distribution
fzx = zeros(size(fx,1),size(fy,2));
sig1 = size_x;
sig2 = size_y;
rho = 0.1;

sig11 = sig1^2;
sig22 = sig2^2;
sig12 = rho * sig1 * sig2;
sig21 = rho * sig1 * sig2;
covar = [sig11, sig12;
         sig21, sig22];

fxi = 0;
fyi = 0;
V = 10.5;
for i = 1:size(fx,1)
    for j = 1:size(fy,2)
        z = ((fx(i,j)-fxi)*(fx(i,j)-fxi))/sig11 + ...
            ((fy(i,j)-fyi)*(fy(i,j)-fyi))/sig22 - ...
            2 * rho * (fx(i,j)-fxi) * (fy(i,j)-fyi)/(sig1*sig2);     
        den = 2 * pi * sig1 * sig2 * sqrt(1-rho^2);
        fzx(i,j) = exp(-z/(2*(1-rho^2)))/den;
    end
end

%% velocity estimation
fzxdot = zeros(size(fx,1),size(fy,2));
sig1 = size_x;
sig2 = size_y;
rho = 0.1;

sig11 = sig1^2;
sig22 = sig2^2;
sig12 = rho * sig1 * sig2;
sig21 = rho * sig1 * sig2;
covar = [sig11, sig12;
         sig21, sig22];

fxi = 0;
fyi = 0;
V = 10.5;
for i = 1:size(fx,1)
    for j = 1:size(fy,2)
        z = ((fx(i,j)-fxi)*(fx(i,j)-fxi))/sig11 + ...
            ((fy(i,j)-fyi)*(fy(i,j)-fyi))/sig22 - ...
            2 * rho * (fx(i,j)-fxi) * (fy(i,j)-fyi)/(sig1*sig2);     
        den = 2 * pi * sig1 * sig2 * sqrt(1-rho^2);
        fzxdot(i,j) = exp(-z/(2*(1-rho^2)))/den;
    end
end

%% final result 
fz = fzx + fzxdot;
surf(fx,fy,fz)
xlabel('x')
ylabel('y')


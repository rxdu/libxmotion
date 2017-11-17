close all; clear; clc;

Pi = [0,0];
step = 0.1;
[fx,fy] = meshgrid(-5:step:5);
fz = zeros(size(fx,1),size(fy,2));

V = 0.5;
for i = 1:size(fx,1)
    for j = 1:size(fy,2)
        fz(i,j) = expm(-(fx(i,j)*fx(i,j) + fy(i,j)*fy(i,j))/(2*V))/(2*pi*V);
    end
end
surf(fx,fy,fz)


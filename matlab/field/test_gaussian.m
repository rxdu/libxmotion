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

figure
surf(fx,fy,fz,'FaceColor','interp',...
   'EdgeColor','none')
%contour(fx,fy,fz)

[c,s]=wavedec2(fz,2,'haar');

[H1,V1,D1] = detcoef2('all',c,s,1);
A1 = appcoef2(c,s,'haar',1); 
V1img = wcodemat(V1,255,'mat',1);
H1img = wcodemat(H1,255,'mat',1);
D1img = wcodemat(D1,255,'mat',1);
A1img = wcodemat(A1,255,'mat',1);

figure
imagesc(A1img);
colormap pink(255);
title('Approximation Coef. of Level 1');
% main script
close all; clear; clc;

% create an empty 2D surface 
step = 0.5;
xmin = -5;
xmax = 15;
ymin = -15;
ymax = 15;
[fx,fy] = meshgrid(xmin:step:xmax, ymin:step:ymax);

% construct typical four-way intersection 
field = zeros(size(fx));
for x = 1:size(fx,1)
    for y = 1:size(fx,2)
        loc_x = xmin + step*(x-1);
        loc_y = ymin + step*(y-1);
        field(x,y) = threat_map(loc_x,loc_y, 0,0,0,0);
    end
end

surf(fx,fy,field)
%contour(fx,fy,field)
pbaspect([1 1 1])
xlabel('x')
ylabel('y')
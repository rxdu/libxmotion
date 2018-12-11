% main script
close all; clear; clc;

% create an empty 2D surface 
step = 0.5;
[fx,fy] = meshgrid(-15:step:15);

% road parameters
road_width = 3.7;

% construct typical four-way intersection 
field = zeros(size(fx));
for x = 1:size(fx,1)
    for y = 1:size(fy,1)
        loc_x = -15 + step*(x-1);
        loc_y = -15 + step*(y-1);
        field(x,y) = threat_map(loc_x,loc_y, 0,0,15,15);
    end
end

surf(fx,fy,field)
%contour(fx,fy,field)
%zlim([0, 15])
pbaspect([1 1 1])
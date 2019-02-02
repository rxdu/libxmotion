% main script
close all; clear; clc;

% create an empty 2D surface 
step = 0.5;
[fx,fy] = meshgrid(-15:step:15);

% road parameters
road_width = 3.7;

% construct typical four-way intersection 
fc_4int = zeros(size(fx));
for x = 1:size(fx,1)
    for y = 1:size(fy,1)
        loc_x = -15 + step*(x-1);
        loc_y = -15 + step*(y-1);
        if (loc_x > -road_width && loc_x < road_width) || (loc_y > -road_width && loc_y < road_width)
            fc_4int(x,y) = fc_4int(x,y) + 1.0;
        end
    end
end

% combine all roads
fc_road = fc_4int;
% combine all collision
fc = fc_road;

surf(fx,fy,fc)
zlim([0, 15])
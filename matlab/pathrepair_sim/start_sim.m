% clear workspace
close all;
clear;
clc;

% start simulation
sdim = [50,50,5];
s = create_3d_space(sdim);

os = add_obstacle_to_space(s, 0.1)

display_2d_map(os)
display_3d_space(os)
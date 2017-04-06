% clear workspace
close all;
clear;
clc;

% start simulation
sdim = [20,20,20];
s = create_3d_space(sdim);

os = add_obstacle_to_space(s, 0.2)

display_3d_space(os)
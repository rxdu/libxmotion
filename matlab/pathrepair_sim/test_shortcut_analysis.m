%% shortcut analysis
close all;
clear;
clc;

ssa = create_3d_space([5, 5, 5]);
ssao = add_obstacle_to_space(ssa, 0.2);
ssamap = create_2d_map(ssao);
sagraph = create_graph(ssamap);
shortcut_analysis(ssamap, sagraph,5,1);

display_2d_map(ssamap)

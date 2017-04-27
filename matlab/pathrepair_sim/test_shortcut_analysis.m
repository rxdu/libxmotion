%% shortcut analysis
close all;
clear;
clc;

ssa = create_3d_space([5, 5, 5]);
ssao = add_obstacle_to_space(ssa, 0.2);
ssamap = create_2d_map(ssao);
sagraph = create_graph(ssamap);
sa_res = shortcut_analysis(ssamap, sagraph,5,1);
[path,dist] = search_2d_path(sagraph,1,25);
dist

ctg = [];
for z = 1:1:sa_res.map.z_size
    for y = 1:1:sa_res.map.y_size
        for x = 1:1:sa_res.map.x_size
            ctg = [ctg sa_res.map.voxels{x,y,z}.cost_to_goal];
        end
    end
end
ctg
display_2d_map(ssamap,path)

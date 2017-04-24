% clear workspace
close all;
clear;
clc;

ssensor = create_3d_space([30 30 10]);
ssensoro = add_obstacle_to_space(ssensor, 0.05);

sensor_data.sensor_pos = [3 3 10];
sensor_data.sensor_angle = pi/3;
sensor_data.range = 8.0;
sensor_data.aov_h = pi*5/12;
sensor_data.aov_v = pi/3;
sensor_data.res = pi/36;

sensor_cor = ssensoro.voxels{sensor_data.sensor_pos(1),...
            sensor_data.sensor_pos(2), ...
            sensor_data.sensor_pos(3)}.center_pos;
beta = sensor_data.sensor_angle;
alpha1 = sensor_data.sensor_angle + sensor_data.aov_h/2;
alpha2 = sensor_data.sensor_angle - sensor_data.aov_h/2;
left_limit = sensor_cor(1:2) + [sin(alpha1) cos(alpha1)] * sensor_data.range
right_limit = sensor_cor(1:2) + [sin(alpha2) cos(alpha2)] * sensor_data.range
ll_idx = get_voxel_free_index([left_limit sensor_cor(3)])
rl_idx = get_voxel_free_index([right_limit sensor_cor(3)])

pp_half_layer_num = floor(sensor_data.range*tan(sensor_data.aov_v/2));
llu_idx = [ll_idx(1) ll_idx(2) ll_idx(3) + pp_half_layer_num];
rlu_idx = [rl_idx(1) rl_idx(2) rl_idx(3) + pp_half_layer_num];
lll_idx = [ll_idx(1) ll_idx(2) ll_idx(3) - pp_half_layer_num];
rll_idx = [rl_idx(1) rl_idx(2) rl_idx(3) - pp_half_layer_num];

% calculate boundary points
%ll_pos = ssensoro.voxels{ll_idx(1),ll_idx(2), ll_idx(3)}.center_pos;
%rl_pos = ssensoro.voxels{rl_idx(1),rl_idx(2), rl_idx(3)}.center_pos;
ll_pos = calc_center_pos([ll_idx(1),ll_idx(2), ll_idx(3)]);
rl_pos = calc_center_pos([rl_idx(1),rl_idx(2), rl_idx(3)]);

%llu_pos = ssensoro.voxels{llu_idx(1),llu_idx(2), llu_idx(3)}.center_pos;
%rlu_pos = ssensoro.voxels{rlu_idx(1),rlu_idx(2), rlu_idx(3)}.center_pos;
llu_pos = calc_center_pos([llu_idx(1),llu_idx(2), llu_idx(3)]);
rlu_pos = calc_center_pos([rlu_idx(1),rlu_idx(2), rlu_idx(3)]);

%lll_pos = ssensoro.voxels{lll_idx(1),lll_idx(2), lll_idx(3)}.center_pos;
%rll_pos = ssensoro.voxels{rll_idx(1),rll_idx(2), rll_idx(3)}.center_pos;
lll_pos = calc_center_pos([lll_idx(1),lll_idx(2), lll_idx(3)]);
rll_pos = calc_center_pos([rll_idx(1),rll_idx(2), rll_idx(3)]);

sensor_data.projection_plane_points = [ll_pos; rl_pos; llu_pos; rlu_pos; lll_pos; rll_pos];

%% plot data
display_3d_space(ssensoro,sensor_data)

hold on

% projection plane
plot3([ll_pos(1) rl_pos(1)],[ll_pos(2) rl_pos(2)],[ll_pos(3) rl_pos(3)],'b')
plot3([lll_pos(1) llu_pos(1) rlu_pos(1) rll_pos(1) lll_pos(1)],...
      [lll_pos(2) llu_pos(2) rlu_pos(2) rll_pos(2) lll_pos(2)],...
      [lll_pos(3) llu_pos(3) rlu_pos(3) rll_pos(3) lll_pos(3)],'b')

% boundary points
plot3(ll_pos(1),ll_pos(2),ll_pos(3),'b*')
plot3(rl_pos(1),rl_pos(2),rl_pos(3),'b*')

plot3(llu_pos(1),llu_pos(2),llu_pos(3),'b*')
plot3(rlu_pos(1),rlu_pos(2),rlu_pos(3),'b*')

plot3(lll_pos(1),lll_pos(2),lll_pos(3),'b*')
plot3(rll_pos(1),rll_pos(2),rll_pos(3),'b*')

% sensor to boundaries
plot3([ll_pos(1) sensor_cor(1)],[ll_pos(2) sensor_cor(2)],[ll_pos(3) sensor_cor(3)],'b')
plot3([rl_pos(1) sensor_cor(1)],[rl_pos(2) sensor_cor(2)],[rl_pos(3) sensor_cor(3)],'b')

plot3([llu_pos(1) sensor_cor(1)],[llu_pos(2) sensor_cor(2)],[llu_pos(3) sensor_cor(3)],'b')
plot3([rlu_pos(1) sensor_cor(1)],[rlu_pos(2) sensor_cor(2)],[rlu_pos(3) sensor_cor(3)],'b')

plot3([lll_pos(1) sensor_cor(1)],[lll_pos(2) sensor_cor(2)],[lll_pos(3) sensor_cor(3)],'b')
plot3([rll_pos(1) sensor_cor(1)],[rll_pos(2) sensor_cor(2)],[rll_pos(3) sensor_cor(3)],'b')

hold off